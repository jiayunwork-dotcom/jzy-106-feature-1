package api

import (
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"

	"github.com/example/attitude-service/internal/euler"
	"github.com/example/attitude-service/internal/trajectory"
	"github.com/example/attitude-service/internal/validation"
)

// This file is the thin HTTP adapter for stateful trajectories. It only
// parses requests, translates validation/trajectory errors into HTTP
// responses and frames DTOs: all merging, archiving and recomputation lives in
// the trajectory package and every integration step goes through
// integrator.Step, shared with the one-shot endpoint.

func writeTrajectoryError(c *gin.Context, err *trajectory.Error) {
	status := http.StatusBadRequest
	switch err.Cause {
	case trajectory.CauseVersionConflict, trajectory.CausePacketConflict,
		trajectory.CauseTimestampConflict:
		status = http.StatusConflict
	case trajectory.CauseNormDriftExceeded, trajectory.CauseLateOutOfWindow:
		status = http.StatusUnprocessableEntity
	case trajectory.CauseTrajectoryNotFound:
		status = http.StatusNotFound
	}
	c.JSON(status, errorResponse{
		Error:  "轨迹操作被拒绝",
		Cause:  err.Cause,
		Detail: err.Message,
	})
}

func (s *Server) createTrajectory(c *gin.Context) {
	var req createTrajectoryRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error: "请求体不是合法 JSON", Cause: "bad_json", Detail: err.Error(),
		})
		return
	}

	in, err := validation.ValidateTrajectoryInit(req.Q0, req.MaxStepNormDrift, req.StrictDrift)
	if err != nil {
		writeValidationError(c, err)
		return
	}

	tr := s.trajectories.Create(trajectory.Config{
		Initial:          in.Q0,
		MaxStepNormDrift: in.MaxStepNormDrift,
		StrictDrift:      in.StrictDrift,
	})

	c.JSON(http.StatusCreated, gin.H{
		"trajectory_id":         tr.ID(),
		"version":               int64(0),
		"initial_quaternion":    quatArr(in.Q0),
		"norm_drift_threshold":  in.MaxStepNormDrift,
		"strict_drift":          in.StrictDrift,
		"max_late_samples":      trajectory.MaxLateSamples,
		"persistent":            false,
		"note":                  "轨迹仅保存在进程内存中，服务重启后失效",
		"quaternion_convention": convention(),
	})
}

func (s *Server) listTrajectories(c *gin.Context) {
	ids := s.trajectories.IDs()
	if ids == nil {
		ids = []string{}
	}
	c.JSON(http.StatusOK, gin.H{"trajectories": ids})
}

func (s *Server) getTrajectoryTarget(c *gin.Context) (*trajectory.Trajectory, bool) {
	tr, ok := s.trajectories.Get(c.Param("id"))
	if !ok {
		c.JSON(http.StatusNotFound, errorResponse{
			Error:  "轨迹不存在",
			Cause:  string(trajectory.CauseTrajectoryNotFound),
			Detail: "未找到标识为 " + c.Param("id") + " 的轨迹（可能已删除或服务重启导致内存清空）",
		})
		return nil, false
	}
	return tr, true
}

func trajectoryView(info trajectory.Info) trajectoryJSON {
	v := trajectoryJSON{
		ID:                 info.ID,
		Version:            info.Version,
		CreatedAt:          info.CreatedAt.Format("2006-01-02T15:04:05.999999999Z07:00"),
		InitialQuaternion:  quatArr(info.Initial),
		NormDriftThreshold: info.Threshold,
		StrictDrift:        info.StrictDrift,
		SampleCount:        info.SampleCount,
		StepCount:          info.StepCount,
		ElapsedTime:        info.ElapsedTime,
		MaxNormDrift:       info.MaxDrift,
		OpenForAppends:     info.OpenForAppends,
		PacketSeqs:         info.PacketSeqs,
	}
	if v.PacketSeqs == nil {
		v.PacketSeqs = []int64{}
	}
	if info.SampleCount > 0 {
		st, en := info.StartTime, info.EndTime
		v.StartTime = &st
		v.EndTime = &en
	}
	return v
}

func (s *Server) getTrajectory(c *gin.Context) {
	tr, ok := s.getTrajectoryTarget(c)
	if !ok {
		return
	}
	c.JSON(http.StatusOK, gin.H{"trajectory": trajectoryView(tr.Info())})
}

func (s *Server) deleteTrajectory(c *gin.Context) {
	id := c.Param("id")
	if !s.trajectories.Delete(id) {
		c.JSON(http.StatusNotFound, errorResponse{
			Error:  "轨迹不存在",
			Cause:  string(trajectory.CauseTrajectoryNotFound),
			Detail: "未找到标识为 " + id + " 的轨迹",
		})
		return
	}
	c.JSON(http.StatusOK, gin.H{"deleted": id})
}

// packetRates mirrors resolveShapes for a packet payload, without the named
// series option (packets always carry their own samples).
func packetRates(req appendTrajectoryRequest) ([]validation.AngularRate, *errorResponse) {
	switch {
	case req.Samples != nil:
		rates := make([]validation.AngularRate, len(req.Samples))
		for i, sm := range req.Samples {
			rates[i] = validation.AngularRate{T: sm.T, W: sm.W}
		}
		return rates, nil
	case req.Timestamps != nil || req.AngularVelocities != nil:
		if len(req.Timestamps) != len(req.AngularVelocities) {
			return nil, &errorResponse{
				Error:  "角速度采样个数与时间戳个数对不上",
				Cause:  string(validation.CauseLengthMismatch),
				Detail: itoa(len(req.Timestamps)) + " 个时间戳 vs " + itoa(len(req.AngularVelocities)) + " 个角速度采样",
			}
		}
		rates := make([]validation.AngularRate, len(req.Timestamps))
		for i := range req.Timestamps {
			rates[i] = validation.AngularRate{T: req.Timestamps[i], W: req.AngularVelocities[i]}
		}
		return rates, nil
	default:
		return nil, &errorResponse{
			Error:  "缺少角速度序列",
			Cause:  string(validation.CauseEmptySequence),
			Detail: "需提供 samples，或 timestamps+angular_velocities",
		}
	}
}

func appendWarnings(ws []trajectory.Warning, sampleCount int) []string {
	// Same flat string list (and same wording) as the one-shot response; the
	// global step number is embedded in each message.
	out := make([]string, 0, len(ws)+1)
	for _, w := range ws {
		out = append(out, w.Message)
	}
	if sampleCount == 1 {
		out = append(out, "序列仅含 1 个采样，没有可推进的时间步，姿态保持为初始姿态")
	}
	return out
}

func (s *Server) appendTrajectory(c *gin.Context) {
	tr, ok := s.getTrajectoryTarget(c)
	if !ok {
		return
	}

	var req appendTrajectoryRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error: "请求体不是合法 JSON", Cause: "bad_json", Detail: err.Error(),
		})
		return
	}

	rates, bad := packetRates(req)
	if bad != nil {
		c.JSON(http.StatusBadRequest, bad)
		return
	}
	samples, verr := validation.ValidatePacket(req.PacketSeq, rates)
	if verr != nil {
		writeValidationError(c, verr)
		return
	}

	res, terr := tr.Append(*req.PacketSeq, req.ExpectedVersion, samples)
	if terr != nil {
		writeTrajectoryError(c, terr)
		return
	}

	ea := euler.FromQuaternion(res.Final.W, res.Final.X, res.Final.Y, res.Final.Z)
	c.JSON(http.StatusOK, gin.H{
		"quaternion_convention": convention(),
		"trajectory_id":         tr.ID(),
		"packet_seq":            *req.PacketSeq,
		"version":               res.Version,
		"duplicate_packet":      res.Duplicate,
		"inserted_samples":      res.InsertedSamples,
		"recomputed_steps":      res.RecomputedSteps,
		"initial_quaternion":    quatArr(tr.Config().Initial),
		"final_quaternion":      quatArr(res.Final),
		"euler_angles": eulerJSON{
			Roll: ea.Roll, Pitch: ea.Pitch, Yaw: ea.Yaw,
			RollDeg: ea.Roll * radToDeg, PitchDeg: ea.Pitch * radToDeg, YawDeg: ea.Yaw * radToDeg,
			Singular: ea.Singular, NearSingular: ea.NearSingular, Note: ea.Note,
		},
		"sample_count":               res.SampleCount,
		"step_count":                 res.StepCount,
		"elapsed_time_s":             res.ElapsedTime,
		"max_norm_drift":             res.MaxDrift,
		"norm_drift_threshold":       tr.Config().MaxStepNormDrift,
		"normalized_after_each_step": true,
		"warnings":                   appendWarnings(res.Warnings, res.SampleCount),
	})
}

func (s *Server) trajectoryAttitude(c *gin.Context) {
	tr, ok := s.getTrajectoryTarget(c)
	if !ok {
		return
	}
	raw := c.Query("t")
	if raw == "" {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error:  "缺少查询时刻",
			Cause:  string(trajectory.CauseInvalidTimestamp),
			Detail: "需以查询参数 t=<秒> 指定轨迹覆盖范围内的时刻",
		})
		return
	}
	at, err := strconv.ParseFloat(raw, 64)
	if err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error:  "查询时刻不是合法数字",
			Cause:  string(trajectory.CauseInvalidTimestamp),
			Detail: "t=" + raw + " 无法解析为浮点数",
		})
		return
	}

	p, terr := tr.AttitudeAt(at)
	if terr != nil {
		writeTrajectoryError(c, terr)
		return
	}
	ea := euler.FromQuaternion(p.Quaternion.W, p.Quaternion.X, p.Quaternion.Y, p.Quaternion.Z)
	c.JSON(http.StatusOK, gin.H{
		"trajectory_id": tr.ID(),
		"t":             p.T,
		"exact_sample":  p.ExactSample,
		"bracket_step":  p.BracketStep,
		"quaternion":    quatArr(p.Quaternion),
		"euler_angles": eulerJSON{
			Roll: ea.Roll, Pitch: ea.Pitch, Yaw: ea.Yaw,
			RollDeg: ea.Roll * radToDeg, PitchDeg: ea.Pitch * radToDeg, YawDeg: ea.Yaw * radToDeg,
			Singular: ea.Singular, NearSingular: ea.NearSingular, Note: ea.Note,
		},
		"interpolation": "linear in angular velocity between bracketing samples; RK4 advance",
	})
}
