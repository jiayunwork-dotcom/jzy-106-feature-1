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

// ValidateInitial checks an initial attitude quaternion: finite components and
// unit norm. It returns the normalized quaternion. The rules are exactly the
// q0 rules of ValidateIntegrate, so trajectory creation cannot diverge from the
// one-shot endpoint.
func ValidateInitial(q0 [4]float64) (quaternion.Q, error) {
	q := quaternion.Q{W: q0[0], X: q0[1], Y: q0[2], Z: q0[3]}
	if !quaternion.IsFinite(q) {
		return quaternion.Q{}, NewError(CauseInvalidQuaternion,
			"初始四元数含 NaN 或无穷大分量")
	}
	if !quaternion.IsUnit(q, UnitTolerance) {
		return quaternion.Q{}, NewErrorf(CauseInvalidQuaternion,
			"初始四元数不是单位四元数: |q0|=%g, 要求 | |q0|-1 | <= %g",
			quaternion.Norm(q), UnitTolerance)
	}
	q0n, _ := quaternion.Normalized(q)
	return q0n, nil
}

// ValidateThreshold checks an explicit per-step drift threshold. A nil pointer
// selects the built-in default. This is the same rule ValidateIntegrate uses.
func ValidateThreshold(maxDrift *float64) (float64, error) {
	threshold := integrator.DefaultMaxStepNormDrift
	if maxDrift != nil {
		if math.IsNaN(*maxDrift) || math.IsInf(*maxDrift, 0) || *maxDrift <= 0 {
			return 0, NewError(CauseInvalidThreshold,
				"单步范数漂移阈值必须是正数")
		}
		threshold = *maxDrift
	}
	return threshold, nil
}

// checkRate rejects NaN/Inf in one sample and, for i > 0, a non-positive time
// step against the previous timestamp. It is the single shared per-sample rule
// for every code path that accepts a rate series. The named-series endpoint
// historically used a shorter step message; styleSeries preserves it exactly.
func checkRate(prevT float64, r AngularRate, i int, styleSeries bool) error {
	if math.IsNaN(r.T) || math.IsInf(r.T, 0) {
		return NewErrorf(CauseInvalidTimestamp,
			"第 %d 个采样的时间戳为 NaN 或无穷大", i)
	}
	if !isFinite3(r.W) {
		return NewErrorf(CauseInvalidRate,
			"第 %d 个采样的角速度含 NaN 或无穷大分量", i)
	}
	if i > 0 {
		if h := r.T - prevT; h <= 0 {
			if styleSeries {
				return NewErrorf(CauseNonPositiveStep,
					"第 %d 个采样的时间步长非正: dt=%.12g", i, h)
			}
			return NewErrorf(CauseNonPositiveStep,
				"第 %d 个采样的时间步长非正: dt=%.12g (时间戳必须严格递增)", i, h)
		}
	}
	return nil
}

// validateRates converts request rates into integrator samples while applying
// every per-sample rule of the one-shot endpoint, keeping its wording stable.
func validateRates(rates []AngularRate) ([]integrator.Sample, error) {
	samples := make([]integrator.Sample, len(rates))
	for i, r := range rates {
		var prevT float64
		if i > 0 {
			prevT = rates[i-1].T
		}
		if err := checkRate(prevT, r, i, false); err != nil {
			return nil, err
		}
		samples[i] = integrator.Sample{T: r.T, W: r.W}
	}
	return samples, nil
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

	q0n, err := ValidateInitial(q0)
	if err != nil {
		return in, err
	}

	if len(rates) == 0 {
		return in, NewError(CauseEmptySequence, "角速度序列为空: 至少需要一个采样")
	}

	samples, err := validateRates(rates)
	if err != nil {
		return in, err
	}

	threshold, err := ValidateThreshold(maxDrift)
	if err != nil {
		return in, err
	}

	in.Q0 = q0n
	in.Samples = samples
	in.MaxStepNormDrift = threshold
	in.StrictDrift = strict
	return in, nil
}

// ValidatePacket checks one trajectory append packet: a non-empty, finite,
// strictly-time-ordered batch of samples with no duplicate timestamps. It
// applies the SAME per-sample rules as ValidateIntegrate (same causes and
// messages), so packet input can never bypass the one-shot validation.
func ValidatePacket(rates []AngularRate) ([]integrator.Sample, error) {
	if len(rates) == 0 {
		return nil, NewError(CauseEmptySequence, "数据包采样为空: 每个数据包至少需要一个采样")
	}
	return validateRates(rates)
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
		var prevT float64
		if i > 0 {
			prevT = rates[i-1].T
		}
		if err := checkRate(prevT, r, i, true); err != nil {
			return in, err
		}
		samples[i] = integrator.Sample{T: r.T, W: r.W}
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

	// Stateful-trajectory causes. They describe rejections the one-shot
	// endpoint can never hit, so they are additional, never replacements.
	CauseVersionConflict   Cause = "version_conflict"
	CauseDuplicatePacket   Cause = "duplicate_packet"
	CausePacketConflict    Cause = "packet_conflict"
	CauseLateWindowExpired Cause = "late_window_expired"
	CauseInvalidPacketSeq  Cause = "invalid_packet_seq"
	CauseInvalidTime       Cause = "invalid_time"
	CauseOutsideCoverage   Cause = "outside_coverage"
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
