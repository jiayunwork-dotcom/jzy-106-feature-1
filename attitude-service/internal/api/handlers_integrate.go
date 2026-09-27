package api

import (
	"errors"
	"math"
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/example/attitude-service/internal/euler"
	"github.com/example/attitude-service/internal/integrator"
	"github.com/example/attitude-service/internal/quaternion"
	"github.com/example/attitude-service/internal/validation"
)

const radToDeg = 180.0 / math.Pi

// resolveShapes turns whichever series representation the client used into the
// canonical []AngularRate form. Precedence: named stored series > inline
// samples > parallel timestamps/angular_velocities arrays.
func (s *Server) resolveShapes(
	seriesName string,
	samples []sampleJSON,
	ts []float64,
	ws [][3]float64,
) ([]validation.AngularRate, *errorResponse) {
	switch {
	case seriesName != "":
		stored, ok := s.store.Get(seriesName)
		if !ok {
			return nil, &errorResponse{
				Error: "命名角速度序列不存在", Cause: string(validation.CauseNotFound),
				Detail: "未找到名为 " + seriesName + " 的序列，请先用 PUT /api/v1/series/:name 保存",
			}
		}
		rates := make([]validation.AngularRate, len(stored))
		for i, sm := range stored {
			rates[i] = validation.AngularRate{T: sm.T, W: sm.W}
		}
		return rates, nil

	case samples != nil:
		// An explicitly supplied (even if empty) samples array flows through
		// validation so the "empty sequence" cause and message stay canonical.
		rates := make([]validation.AngularRate, len(samples))
		for i, sm := range samples {
			rates[i] = validation.AngularRate{T: sm.T, W: sm.W}
		}
		return rates, nil

	case ts != nil || ws != nil:
		if len(ts) != len(ws) {
			return nil, &errorResponse{
				Error: "角速度采样个数与时间戳个数对不上", Cause: string(validation.CauseLengthMismatch),
				Detail: plural(len(ts), "个时间戳") + " vs " + plural(len(ws), "个角速度采样"),
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
			Detail: "需提供 samples，或 timestamps+angular_velocities，或已保存的 series 名称",
		}
	}
}

func plural(n int, _ string) string {
	return itoa(n)
}

// itoa renders a non-negative integer without pulling strconv just for errors.
func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	var b [20]byte
	p := len(b)
	for n > 0 {
		p--
		b[p] = byte('0' + n%10)
		n /= 10
	}
	return string(b[p:])
}

func writeValidationError(c *gin.Context, err error) {
	var ve *validation.Error
	if errors.As(err, &ve) {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error: "请求参数非法", Cause: string(ve.Cause), Detail: ve.Message,
		})
		return
	}
	c.JSON(http.StatusBadRequest, errorResponse{
		Error: "请求参数非法", Detail: err.Error(),
	})
}

func convention() conventionJSON {
	return conventionJSON{
		ComponentOrder:    "[w, x, y, z] scalar-first",
		ProductConvention: "Hamilton: ij=k, jk=i, ki=j",
		FrameMapping:      "q rotates vectors from body frame to reference frame: v_n = q ⊗ v_b ⊗ q*",
		KinematicEquation: "q̇ = 1/2 · q ⊗ (0, ω_b); body-frame angular velocity RIGHT-multiplied",
		EulerOrder:        euler.Order,
		AngleUnit:         "euler angles in radians (also *_deg in degrees)",
		Integrator:        "classical RK4, quaternion renormalized after every step",
	}
}

func quatArr(q quaternion.Q) [4]float64 {
	return [4]float64{q.W, q.X, q.Y, q.Z}
}

func (s *Server) integrate(c *gin.Context) {
	var req integrateRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error: "请求体不是合法 JSON", Cause: "bad_json", Detail: err.Error(),
		})
		return
	}

	rates, bad := s.resolveShapes(req.Series, req.Samples, req.Timestamps, req.AngularVelocities)
	if bad != nil {
		c.JSON(http.StatusBadRequest, bad)
		return
	}

	in, err := validation.ValidateIntegrate(req.Q0, rates, req.MaxStepNormDrift, req.StrictDrift)
	if err != nil {
		writeValidationError(c, err)
		return
	}

	res, err := integrator.Simulate(in.Q0, in.Samples, integrator.Options{
		MaxStepNormDrift: in.MaxStepNormDrift,
		StrictDrift:      in.StrictDrift,
	})
	if err != nil {
		switch {
		case errors.Is(err, integrator.ErrDriftThreshold):
			c.JSON(http.StatusUnprocessableEntity, errorResponse{
				Error:  "单步范数漂移超过阈值，已拒绝继续积分",
				Cause:  "norm_drift_exceeded",
				Detail: err.Error(),
			})
		default:
			c.JSON(http.StatusUnprocessableEntity, errorResponse{
				Error: "积分无法继续", Detail: err.Error(),
			})
		}
		return
	}

	ea := euler.FromQuaternion(res.Final.W, res.Final.X, res.Final.Y, res.Final.Z)
	warnings := res.Warnings
	if warnings == nil {
		warnings = []string{}
	}
	if len(in.Samples) == 1 {
		warnings = append(warnings, "序列仅含 1 个采样，没有可推进的时间步，姿态保持为初始姿态")
	}

	c.JSON(http.StatusOK, integrateResponse{
		QuaternionConvention: convention(),
		InitialQuaternion:    quatArr(res.Initial),
		FinalQuaternion:      quatArr(res.Final),
		EulerAngles: eulerJSON{
			Roll: ea.Roll, Pitch: ea.Pitch, Yaw: ea.Yaw,
			RollDeg: ea.Roll * radToDeg, PitchDeg: ea.Pitch * radToDeg, YawDeg: ea.Yaw * radToDeg,
			Singular: ea.Singular, NearSingular: ea.NearSingular, Note: ea.Note,
		},
		SampleCount:        len(in.Samples),
		StepCount:          len(res.Steps),
		ElapsedTime:        res.ElapsedTime,
		MaxNormDrift:       res.MaxDrift,
		NormDriftThreshold: in.MaxStepNormDrift,
		NormalizedEachStep: true,
		Warnings:           warnings,
	})
}
