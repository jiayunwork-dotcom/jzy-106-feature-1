// Package validation validates and translates HTTP-layer request payloads
// into the domain types consumed by the integrator. It owns every rule about
// what constitutes a legal integration request, so that the HTTP handler
// stays thin.
package validation

import (
	"fmt"
	"math"
	"regexp"
	"strings"

	"github.com/example/attitude-service/internal/integrator"
	"github.com/example/attitude-service/internal/quaternion"
)

// UnitTolerance is the maximum allowed | |q0| - 1 | for the initial attitude.
const UnitTolerance = 1e-6

var namePattern = regexp.MustCompile(`^[A-Za-z0-9_.:@-]{1,128}$`)

// AngularRate is one JSON sample: a timestamp plus the body angular velocity.
type AngularRate struct {
	T float64    `json:"t"`
	W [3]float64 `json:"w"`
}

// IntegrateInput is the validated, integration-ready request content.
type IntegrateInput struct {
	Q0               quaternion.Q
	Samples          []integrator.Sample
	MaxStepNormDrift float64
	StrictDrift      bool
}

// SeriesInput is a named, storable angular velocity series.
type SeriesInput struct {
	Name    string
	Samples []integrator.Sample
}

func isFinite3(v [3]float64) bool {
	for _, c := range v {
		if math.IsNaN(c) || math.IsInf(c, 0) {
			return false
		}
	}
	return true
}

// ValidateIntegrate checks every pre-integration rule:
//   - the series must be non-empty;
//   - timestamps and angular velocity samples must be given one-for-one
//     (enforced structurally by the JSON binding and re-checked here);
//   - timestamps must be strictly increasing (every step strictly positive);
//   - no component may be NaN or ±Inf;
//   - the initial quaternion must be a unit quaternion and finite;
//   - an explicit drift threshold must be positive.
//
// The returned error always carries a human-readable Chinese cause and is
// wrapped with a stable Cause code for the API layer.
func ValidateIntegrate(
	q0 [4]float64,
	rates []AngularRate,
	maxDrift *float64,
	strict bool,
) (IntegrateInput, error) {
	var in IntegrateInput

	q := quaternion.Q{W: q0[0], X: q0[1], Y: q0[2], Z: q0[3]}
	if !quaternion.IsFinite(q) {
		return in, NewError(CauseInvalidQuaternion,
			"初始四元数含 NaN 或无穷大分量")
	}
	if !quaternion.IsUnit(q, UnitTolerance) {
		return in, NewErrorf(CauseInvalidQuaternion,
			"初始四元数不是单位四元数: |q0|=%g, 要求 | |q0|-1 | <= %g",
			quaternion.Norm(q), UnitTolerance)
	}
	q0n, _ := quaternion.Normalized(q)

	if len(rates) == 0 {
		return in, NewError(CauseEmptySequence, "角速度序列为空: 至少需要一个采样")
	}

	samples := make([]integrator.Sample, len(rates))
	for i, r := range rates {
		if math.IsNaN(r.T) || math.IsInf(r.T, 0) {
			return in, NewErrorf(CauseInvalidTimestamp,
				"第 %d 个采样的时间戳为 NaN 或无穷大", i)
		}
		if !isFinite3(r.W) {
			return in, NewErrorf(CauseInvalidRate,
				"第 %d 个采样的角速度含 NaN 或无穷大分量", i)
		}
		samples[i] = integrator.Sample{T: r.T, W: r.W}
		if i > 0 {
			h := r.T - rates[i-1].T
			if h <= 0 {
				return in, NewErrorf(CauseNonPositiveStep,
					"第 %d 个采样的时间步长非正: dt=%.12g (时间戳必须严格递增)", i, h)
			}
		}
	}

	threshold := integrator.DefaultMaxStepNormDrift
	if maxDrift != nil {
		if math.IsNaN(*maxDrift) || math.IsInf(*maxDrift, 0) || *maxDrift <= 0 {
			return in, NewError(CauseInvalidThreshold,
				"单步范数漂移阈值必须是正数")
		}
		threshold = *maxDrift
	}

	in.Q0 = q0n
	in.Samples = samples
	in.MaxStepNormDrift = threshold
	in.StrictDrift = strict
	return in, nil
}

// ValidateSeries checks a store/save payload: a legal name and a legal,
// non-empty time series with strictly increasing timestamps.
func ValidateSeries(name string, rates []AngularRate) (SeriesInput, error) {
	var in SeriesInput
	trimmed := strings.TrimSpace(name)
	if !namePattern.MatchString(trimmed) {
		return in, NewError(CauseInvalidName,
			"序列名称非法: 需为 1-128 个字母、数字或 _ . : @ - 字符")
	}
	if len(rates) == 0 {
		return in, NewError(CauseEmptySequence, "角速度序列为空: 至少需要一个采样")
	}
	samples := make([]integrator.Sample, len(rates))
	for i, r := range rates {
		if math.IsNaN(r.T) || math.IsInf(r.T, 0) {
			return in, NewErrorf(CauseInvalidTimestamp,
				"第 %d 个采样的时间戳为 NaN 或无穷大", i)
		}
		if !isFinite3(r.W) {
			return in, NewErrorf(CauseInvalidRate,
				"第 %d 个采样的角速度含 NaN 或无穷大分量", i)
		}
		samples[i] = integrator.Sample{T: r.T, W: r.W}
		if i > 0 && r.T <= rates[i-1].T {
			return in, NewErrorf(CauseNonPositiveStep,
				"第 %d 个采样的时间步长非正: dt=%.12g", i, r.T-rates[i-1].T)
		}
	}
	in.Name = trimmed
	in.Samples = samples
	return in, nil
}

// Cause is a stable machine-readable error cause.
type Cause string

// Error causes returned by validation.
const (
	CauseInvalidQuaternion Cause = "invalid_quaternion"
	CauseEmptySequence     Cause = "empty_sequence"
	CauseLengthMismatch    Cause = "length_mismatch"
	CauseNonPositiveStep   Cause = "non_positive_step"
	CauseInvalidTimestamp  Cause = "invalid_timestamp"
	CauseInvalidRate       Cause = "invalid_rate"
	CauseInvalidThreshold  Cause = "invalid_threshold"
	CauseInvalidName       Cause = "invalid_name"
	CauseNotFound          Cause = "not_found"
)

// Error is a validation error carrying a stable cause and a Chinese message.
type Error struct {
	Cause   Cause
	Message string
}

func (e *Error) Error() string { return fmt.Sprintf("%s: %s", e.Cause, e.Message) }

// NewError builds a validation error.
func NewError(cause Cause, msg string) *Error {
	return &Error{Cause: cause, Message: msg}
}

// NewErrorf builds a formatted validation error.
func NewErrorf(cause Cause, format string, args ...any) *Error {
	return &Error{Cause: cause, Message: fmt.Sprintf(format, args...)}
}
