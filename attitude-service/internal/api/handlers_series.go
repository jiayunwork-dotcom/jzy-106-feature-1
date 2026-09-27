package api

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/example/attitude-service/internal/validation"
)

func (s *Server) seriesFromRequest(name string, req seriesRequest) ([]validation.AngularRate, *errorResponse) {
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
			Error: "缺少角速度序列", Cause: string(validation.CauseEmptySequence),
			Detail: "需提供 samples，或 timestamps+angular_velocities",
		}
	}
}

func (s *Server) saveSeries(c *gin.Context) {
	var req seriesRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Error: "请求体不是合法 JSON", Cause: "bad_json", Detail: err.Error(),
		})
		return
	}
	rates, bad := s.seriesFromRequest(c.Param("name"), req)
	if bad != nil {
		c.JSON(http.StatusBadRequest, bad)
		return
	}
	in, err := validation.ValidateSeries(c.Param("name"), rates)
	if err != nil {
		writeValidationError(c, err)
		return
	}
	s.store.Save(in.Name, in.Samples)
	c.JSON(http.StatusCreated, gin.H{
		"name":         in.Name,
		"sample_count": len(in.Samples),
		"persistent":   false,
		"note":         "序列仅保存在进程内存中，服务重启后失效",
	})
}

func (s *Server) listSeries(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{"series": s.store.Names()})
}

func (s *Server) getSeries(c *gin.Context) {
	name := c.Param("name")
	samples, ok := s.store.Get(name)
	if !ok {
		c.JSON(http.StatusNotFound, errorResponse{
			Error: "命名角速度序列不存在", Cause: string(validation.CauseNotFound),
			Detail: "未找到名为 " + name + " 的序列",
		})
		return
	}
	out := make([]sampleJSON, len(samples))
	for i, sm := range samples {
		out[i] = sampleJSON{T: sm.T, W: sm.W}
	}
	c.JSON(http.StatusOK, gin.H{"name": name, "samples": out})
}

func (s *Server) deleteSeries(c *gin.Context) {
	name := c.Param("name")
	if !s.store.Delete(name) {
		c.JSON(http.StatusNotFound, errorResponse{
			Error: "命名角速度序列不存在", Cause: string(validation.CauseNotFound),
			Detail: "未找到名为 " + name + " 的序列",
		})
		return
	}
	c.JSON(http.StatusOK, gin.H{"deleted": name})
}
