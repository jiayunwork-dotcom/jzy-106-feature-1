// Package api is the thin HTTP layer: it parses requests, drives the
// integrator and assembles responses. It deliberately contains no quaternion
// algebra and no integration math.
package api

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/example/attitude-service/internal/store"
)

// Server bundles the dependencies shared by every handler.
type Server struct {
	store *store.SeriesStore
}

// NewServer constructs the HTTP layer.
func NewServer(st *store.SeriesStore) *Server {
	return &Server{store: st}
}

// Router builds the gin engine with every route wired up.
func (s *Server) Router() *gin.Engine {
	gin.SetMode(gin.ReleaseMode)
	r := gin.New()
	r.Use(gin.Recovery())

	r.GET("/health", s.health)

	v1 := r.Group("/api/v1")
	{
		v1.POST("/attitude/integrate", s.integrate)
		v1.PUT("/series/:name", s.saveSeries)
		v1.GET("/series", s.listSeries)
		v1.GET("/series/:name", s.getSeries)
		v1.DELETE("/series/:name", s.deleteSeries)
	}
	return r
}

// ---- request / response DTOs -------------------------------------------------

// integrateRequest accepts either inline "samples" ({t,w} objects) or the
// parallel "timestamps"+"angular_velocities" arrays. "series" references a
// previously stored named series.
type integrateRequest struct {
	Q0                [4]float64   `json:"q0"`
	Series            string       `json:"series"`
	Samples           []sampleJSON `json:"samples"`
	Timestamps        []float64    `json:"timestamps"`
	AngularVelocities [][3]float64 `json:"angular_velocities"`
	MaxStepNormDrift  *float64     `json:"max_step_norm_drift"`
	StrictDrift       bool         `json:"strict_drift"`
}

type sampleJSON struct {
	T float64    `json:"t"`
	W [3]float64 `json:"w"`
}

type eulerJSON struct {
	Roll         float64 `json:"roll_rad"`
	Pitch        float64 `json:"pitch_rad"`
	Yaw          float64 `json:"yaw_rad"`
	RollDeg      float64 `json:"roll_deg"`
	PitchDeg     float64 `json:"pitch_deg"`
	YawDeg       float64 `json:"yaw_deg"`
	Singular     bool    `json:"singular"`
	NearSingular bool    `json:"near_singular"`
	Note         string  `json:"note,omitempty"`
}

type integrateResponse struct {
	QuaternionConvention conventionJSON `json:"quaternion_convention"`
	InitialQuaternion    [4]float64     `json:"initial_quaternion"`
	FinalQuaternion      [4]float64     `json:"final_quaternion"`
	EulerAngles          eulerJSON      `json:"euler_angles"`
	SampleCount          int            `json:"sample_count"`
	StepCount            int            `json:"step_count"`
	ElapsedTime          float64        `json:"elapsed_time_s"`
	MaxNormDrift         float64        `json:"max_norm_drift"`
	NormDriftThreshold   float64        `json:"norm_drift_threshold"`
	NormalizedEachStep   bool           `json:"normalized_after_each_step"`
	Warnings             []string       `json:"warnings"`
}

type conventionJSON struct {
	ComponentOrder    string `json:"component_order"`
	ProductConvention string `json:"product_convention"`
	FrameMapping      string `json:"frame_mapping"`
	KinematicEquation string `json:"kinematic_equation"`
	EulerOrder        string `json:"euler_order"`
	AngleUnit         string `json:"angle_unit"`
	Integrator        string `json:"integrator"`
}

type errorResponse struct {
	Error  string `json:"error"`
	Cause  string `json:"cause"`
	Detail string `json:"detail"`
}

type seriesRequest struct {
	Samples           []sampleJSON `json:"samples"`
	Timestamps        []float64    `json:"timestamps"`
	AngularVelocities [][3]float64 `json:"angular_velocities"`
}

func (s *Server) health(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{"status": "ok", "service": "attitude-quaternion-integrator"})
}
