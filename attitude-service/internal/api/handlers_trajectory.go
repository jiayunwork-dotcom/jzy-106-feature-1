package api

import (
	"errors"
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"

	"github.com/example/attitude-service/internal/euler"
	"github.com/example/attitude-service/internal/integrator"
	"github.com/example/attitude-service/internal/quaternion"
	"github.com/example/attitude-service/internal/trajectory"
	"github.com/example/attitude-service/internal/validation"
)

// attitudeView groups the kernel-independent pieces shared by the one-shot
// integration response and every trajectory append response, so both surfaces
// render the same fields with identical meanings.
type attitudeView struct {
	Initial         quaternion.Q
	Final           quaternion.Q
	SampleCount     int
	StepCount       int
	ElapsedTime     float64
	MaxNormDrift    float64
	NormDriftThresh float64
	Warnings        []string
}

func eulerView(q quaternion.Q) eulerJSON {
	ea := euler.FromQuaternion(q.W, q.X, q.Y, q.Z)
	return eulerJSON{
		Roll: ea.Roll, Pitch: ea.Pitch, Yaw: ea.Yaw,
		RollDeg: ea.Roll * radToDeg, PitchDeg: ea.Pitch * radToDeg, YawDeg: ea.Yaw * radToDeg,
		Singular: ea.Singular, NearSingular: ea.NearSingular, Note: ea.Note,
	}
}

func (s *Server) attitudeResponse(v attitudeView) integrateResponse {
	warnings := v.Warnings
	if warnings == nil {
		warnings = []string{}
	}
	return integrateResponse{
		QuaternionConvention: convention(),
		InitialQuaternion:    quatArr(v.Initial),
		FinalQuaternion:      quatArr(v.Final),
		EulerAngles:          eulerView(v.Final),
		SampleCount:          v.SampleCount,
		StepCount:            v.StepCount,
		ElapsedTime:          v.ElapsedTime,
		MaxNormDrift:         v.MaxNormDrift,
		NormDriftThreshold:   v.NormDriftThresh,
		NormalizedEachStep:   true,
		Warnings:             warnings,
	}
}

// ---- request DTOs ------------------------------------------------------------

type createTrajectoryRequest struct {
	Q0                [4]float64   `json:"q0"`
	Samples           []sampleJSON `json:"samples"`
	Timestamps        []float64    `json:"timestamps"`
	AngularVelocities [][3]float64 `json:"angular_velocities"`
	MaxStepNormDrift  *float64     `json:"max_step_norm_drift"`
	StrictDrift       bool         `json:"strict_drift"`
}

type appendPacketRequest struct {
	Seq               *int64       `json:"seq"`
	Samples           []sampleJSON `json:"samples"`
	Timestamps        []float64    `json:"timestamps"`
	AngularVelocities [][3]float64 `json:"angular_velocities"`
	// ExpectedVersion, when present, is the caller's last observed trajectory
	// version; an append that races another writer fails with 409 instead of
	// being applied.
	ExpectedVersion *int64 `json:"expected_version"`
}

type trajectoryJSON struct {
	ID                 string     `json:"id"`
	Version            int64      `json:"version"`
	InitialQuaternion  [4]float64 `json:"initial_quaternion"`
	StrictDrift        bool       `json:"strict_drift"`
	SampleCount        int        `json:"sample_count"`
	StepCount          int        `json:"step_count"`
	StartTime          *float64   `json:"start_time_s,omitempty"`
	EndTime            *float64   `json:"end_time_s,omitempty"`
	FinalQuaternion    [4]float64 `json:"final_quaternion"`
	EulerAngles        eulerJSON  `json:"euler_angles"`
	MaxNormDrift       float64    `json:"max_norm_drift"`
	NormDriftThreshold float64    `json:"norm_drift_threshold"`
	Warnings           []string   `json:"warnings"`
	AppliedPacketCount int        `json:"applied_packet_count"`
	Persistent         bool       `json:"persistent"`
}

// ---- handlers ---------------------------------------------------------------

func (s *Server) createTrajectory(c *gin.Context) {
	var req createTrajectoryRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error: "请求体不是合法 JSON", Cause: "bad_json", Detail: err.Error(),
		})
		return
	}

	q0, err := validation.ValidateInitialQuaternion(req.Q0)
	if err != nil {
		writeValidationError(c, err)
		return
	}
	threshold, err := validation.ValidateThreshold(req.MaxStepNormDrift)
	if err != nil {
		writeValidationError(c, err)
		return
	}

	cfg := trajectory.Config{
		Q0:               q0,
		MaxStepNormDrift: threshold,
		StrictDrift:      req.StrictDrift,
	}
	id, st, err := s.trajectories.Open(cfg)
	if err != nil {
		c.JSON(http.StatusInternalServerError, errorResponse{
			Error: "无法创建轨迹", Detail: err.Error(),
		})
		return
	}

	c.JSON(http.StatusCreated, gin.H{
		"id":                   id,
		"version":              st.Version,
		"initial_quaternion":   quatArr(st.Initial),
		"norm_drift_threshold": st.MaxStepNormDrift,
		"strict_drift":         st.StrictDrift,
		"final_quaternion":     quatArr(st.Final),
		"euler_angles":         eulerView(st.Final),
		"sample_count":         0,
		"step_count":           0,
		"max_norm_drift":       0.0,
		"warnings":             []string{},
		"persistent":           false,
		"note":                 "轨迹仅保存在进程内存中，服务重启后失效",
	})
}

func (s *Server) getTrajectory(c *gin.Context) {
	st, ok := s.trajectories.Get(c.Param("id"))
	if !ok {
		trajectoryNotFound(c, c.Param("id"))
		return
	}
	c.JSON(http.StatusOK, s.trajectoryView(st))
}

func (s *Server) listTrajectories(c *gin.Context) {
	states := s.trajectories.List()
	out := make([]trajectoryJSON, len(states))
	for i, st := range states {
		out[i] = s.trajectoryView(st)
	}
	c.JSON(http.StatusOK, gin.H{"trajectories": out})
}

func (s *Server) closeTrajectory(c *gin.Context) {
	id := c.Param("id")
	if !s.trajectories.Close(id) {
		trajectoryNotFound(c, id)
		return
	}
	c.JSON(http.StatusOK, gin.H{"closed": id, "deleted": true})
}

func (s *Server) appendPacket(c *gin.Context) {
	id := c.Param("id")

	var req appendPacketRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error: "请求体不是合法 JSON", Cause: "bad_json", Detail: err.Error(),
		})
		return
	}
	if req.Seq == nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error:  "请求参数非法",
			Cause:  string(validation.CauseInvalidPacket),
			Detail: "缺少包序号字段 seq（非负整数）",
		})
		return
	}
	if *req.Seq < 0 {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error:  "请求参数非法",
			Cause:  string(validation.CauseInvalidPacket),
			Detail: "包序号 seq 必须是非负整数",
		})
		return
	}

	rates, bad := s.resolvePacketShapes(req.Samples, req.Timestamps, req.AngularVelocities)
	if bad != nil {
		c.JSON(http.StatusBadRequest, bad)
		return
	}
	// The same per-sample rules as the one-shot endpoint: NaN/Inf rejected,
	// strictly increasing timestamps inside the packet (covers packet-internal
	// duplicate timestamps and non-positive steps).
	samples, verr := validation.ValidateRateSequence(rates)
	if verr != nil {
		writeValidationError(c, verr)
		return
	}

	expected := int64(-1)
	if req.ExpectedVersion != nil {
		expected = *req.ExpectedVersion
	}
	out, err := s.trajectories.Append(id, trajectory.Packet{
		Seq:     *req.Seq,
		Samples: samples,
	}, expected)
	if err != nil {
		writeTrajectoryError(c, err)
		return
	}

	// Snapshot taken atomically with the append inside the trajectory lock, so
	// version and tip can never come from different appends.
	st := out.State

	status := http.StatusOK
	result := gin.H{
		"trajectory_id":    id,
		"version":          out.Version,
		"seq":              *req.Seq,
		"duplicate":        out.Duplicate,
		"recomputed_steps": out.RecomputedSteps,
	}
	if out.Duplicate {
		status = http.StatusOK
		result["status"] = "duplicate"
		result["detail"] = "相同包序号且内容一致，视为重传，未重复施加"
	} else {
		result["status"] = "applied"
	}

	// The attitude payload matches the one-shot integration response in field
	// names and meanings.
	warnings := append([]string(nil), st.Warnings...)
	if st.SampleCount == 1 {
		warnings = append(warnings, "序列仅含 1 个采样，没有可推进的时间步，姿态保持为初始姿态")
	}
	view := s.attitudeResponse(attitudeView{
		Initial:         st.Initial,
		Final:           st.Final,
		SampleCount:     st.SampleCount,
		StepCount:       st.StepCount,
		ElapsedTime:     st.EndTime - st.StartTime,
		MaxNormDrift:    st.MaxNormDrift,
		NormDriftThresh: st.MaxStepNormDrift,
		Warnings:        warnings,
	})
	// Convenience copies of the current tip at the top level; "result" carries
	// the complete one-shot-shaped payload with identical field meanings.
	result["final_quaternion"] = view.FinalQuaternion
	result["euler_angles"] = view.EulerAngles
	result["sample_count"] = view.SampleCount
	result["step_count"] = view.StepCount
	result["elapsed_time_s"] = view.ElapsedTime
	result["max_norm_drift"] = view.MaxNormDrift
	result["norm_drift_threshold"] = view.NormDriftThreshold
	result["warnings"] = view.Warnings
	result["result"] = view
	c.JSON(status, result)
}

func (s *Server) queryAttitude(c *gin.Context) {
	id := c.Param("id")
	tStr := c.Query("t")
	if tStr == "" {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error: "缺少查询参数", Cause: "invalid_timestamp",
			Detail: "需通过 ?t=<秒> 指定查询时刻",
		})
		return
	}
	t, err := strconv.ParseFloat(tStr, 64)
	if err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error: "查询参数非法", Cause: "invalid_timestamp",
			Detail: "t 必须是秒为单位的有限数字: " + tStr,
		})
		return
	}

	pt, terr := s.trajectories.AttitudeAt(id, t)
	if terr != nil {
		writeTrajectoryError(c, terr)
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"trajectory_id":           id,
		"t":                       pt.T,
		"quaternion":              quatArr(pt.Quaternion),
		"quaternion_convention":   convention(),
		"euler_angles":            eulerView(pt.Quaternion),
		"interpolated":            pt.Interpolated,
		"global_step":             pt.StepIndex,
		"partial_step_norm_drift": pt.NormDrift,
	})
}

// ---- shared plumbing ---------------------------------------------------------

func (s *Server) resolvePacketShapes(
	inline []sampleJSON,
	ts []float64,
	ws [][3]float64,
) ([]validation.AngularRate, *errorResponse) {
	switch {
	case inline != nil:
		rates := make([]validation.AngularRate, len(inline))
		for i, sm := range inline {
			rates[i] = validation.AngularRate{T: sm.T, W: sm.W}
		}
		return rates, nil
	case ts != nil || ws != nil:
		if len(ts) != len(ws) {
			return nil, &errorResponse{
				Error:  "角速度采样个数与时间戳个数对不上",
				Cause:  string(validation.CauseLengthMismatch),
				Detail: itoa(len(ts)) + " 个时间戳 vs " + itoa(len(ws)) + " 个角速度采样",
			}
		}
		rates := make([]validation.AngularRate, len(ts))
		for i := range ts {
			rates[i] = validation.AngularRate{T: ts[i], W: ws[i]}
		}
		return rates, nil
	default:
		return nil, &errorResponse{
			Error: "缺少角速度序列", Cause: string(validation.CauseEmptySequence),
			Detail: "需提供 samples，或 timestamps+angular_velocities",
		}
	}
}

func (s *Server) trajectoryView(st trajectory.State) trajectoryJSON {
	j := trajectoryJSON{
		ID:                 st.ID,
		Version:            st.Version,
		InitialQuaternion:  quatArr(st.Initial),
		StrictDrift:        st.StrictDrift,
		SampleCount:        st.SampleCount,
		StepCount:          st.StepCount,
		FinalQuaternion:    quatArr(st.Final),
		EulerAngles:        eulerView(st.Final),
		MaxNormDrift:       st.MaxNormDrift,
		NormDriftThreshold: st.MaxStepNormDrift,
		Warnings:           st.Warnings,
		AppliedPacketCount: st.AppliedPacketCount,
		Persistent:         false,
	}
	if j.Warnings == nil {
		j.Warnings = []string{}
	}
	if st.SampleCount > 0 {
		start, end := st.StartTime, st.EndTime
		j.StartTime = &start
		j.EndTime = &end
	}
	return j
}

func trajectoryNotFound(c *gin.Context, id string) {
	c.JSON(http.StatusNotFound, errorResponse{
		Error: "轨迹不存在", Cause: string(validation.CauseTrajectoryNotFound),
		Detail: "未找到轨迹 " + id + "（可能从未创建或已关闭删除；轨迹仅存内存，重启也会丢失）",
	})
}

// writeTrajectoryError maps trajectory rejections to HTTP codes:
// validation-class causes -> 400, optimistic/content conflicts -> 409,
// strict drift / out-of-window late packets -> 422, missing trajectory -> 404.
func writeTrajectoryError(c *gin.Context, err error) {
	var te *trajectory.Error
	if !errors.As(err, &te) {
		// Integrator errors (e.g. strict drift) can escape the trajectory
		// package unwrapped.
		if errors.Is(err, integrator.ErrDriftThreshold) {
			c.JSON(http.StatusUnprocessableEntity, errorResponse{
				Error: "单步范数漂移超过阈值，已拒绝整包，轨迹回滚到收包前状态",
				Cause: "norm_drift_exceeded", Detail: err.Error(),
			})
			return
		}
		c.JSON(http.StatusUnprocessableEntity, errorResponse{
			Error: "积分无法继续", Detail: err.Error(),
		})
		return
	}

	switch te.Cause {
	case validation.CauseTrajectoryNotFound:
		c.JSON(http.StatusNotFound, errorResponse{
			Error: "轨迹不存在", Cause: string(te.Cause), Detail: te.Message,
		})
	case validation.CauseVersionConflict, validation.CausePacketConflict,
		validation.CauseDuplicateTimestamp:
		c.JSON(http.StatusConflict, errorResponse{
			Error: "包与轨迹当前状态冲突，未生效", Cause: string(te.Cause), Detail: te.Message,
		})
	case validation.CauseLatePacketTooOld, validation.CauseOutOfRangeQuery:
		c.JSON(http.StatusUnprocessableEntity, errorResponse{
			Error: "请求超出轨迹可处理范围", Cause: string(te.Cause), Detail: te.Message,
		})
	default:
		c.JSON(http.StatusBadRequest, errorResponse{
			Error: "请求参数非法", Cause: string(te.Cause), Detail: te.Message,
		})
	}
}
