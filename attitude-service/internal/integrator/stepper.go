package integrator

import (
	"math"

	"github.com/example/attitude-service/internal/quaternion"
)

// Stepper advances an attitude quaternion one RK4 step at a time. It is the
// single incremental core shared by:
//   - Simulate (one-shot integration over a complete series), and
//   - stateful trajectories (packet-by-packet append and suffix recompute).
//
// Because every step — whether executed as part of a one-shot run or appended
// across packet boundaries — flows through Stepper.Advance with the exact same
// floating-point operations, a series split into packets must terminate at a
// quaternion that is bit-for-bit identical to the one-shot result.
type Stepper struct {
	q         quaternion.Q
	threshold float64
	strict    bool
	nextStep  int // 1-based index of the next step performed
	maxDrift  float64
	warnings  []string
}

// NewStepper starts an incremental integration at q0, applying the same drift
// policy Options as Simulate (a non-positive threshold selects the default).
func NewStepper(q0 quaternion.Q, opts Options) *Stepper {
	threshold := opts.MaxStepNormDrift
	if threshold <= 0 {
		threshold = DefaultMaxStepNormDrift
	}
	return &Stepper{
		q:         q0,
		threshold: threshold,
		strict:    opts.StrictDrift,
		nextStep:  1,
	}
}

// Restore repositions the stepper at an archived point of a longer run: the
// attitude q reached after stepIndex steps, together with the diagnostics
// accumulated up to that point. The next Advance is considered global step
// stepIndex+1, so drift errors and warning texts carry whole-trajectory step
// numbers. It is used to recompute a suffix from the nearest archive without
// replaying the whole series. Passed-in warnings are defensively copied.
func (s *Stepper) Restore(q quaternion.Q, stepIndex int, maxDrift float64, warnings []string) {
	s.q = q
	s.nextStep = stepIndex + 1
	s.maxDrift = maxDrift
	s.warnings = cloneWarnings(warnings)
}

// Threshold reports the effective per-step drift threshold.
func (s *Stepper) Threshold() float64 { return s.threshold }

// Attitude returns the post-normalization attitude after the last Advance.
func (s *Stepper) Attitude() quaternion.Q { return s.q }

// StepCount reports how many steps have been performed so far.
func (s *Stepper) StepCount() int { return s.nextStep - 1 }

// MaxDrift reports the largest per-step ||q|-1| seen over the whole run.
func (s *Stepper) MaxDrift() float64 { return s.maxDrift }

// Warnings returns the (at most maxWarnings) accumulated warning texts.
func (s *Stepper) Warnings() []string { return s.warnings }

// Advance performs one RK4 step from sample a to sample b, interpolating the
// body angular velocity linearly between them exactly as Simulate would for an
// adjacent pair in a one-shot series. It records the step's diagnostics,
// enforces the drift policy (warn-and-continue or strict rejection) and
// renormalizes the quaternion.
func (s *Stepper) Advance(a, b Sample) (StepRecord, error) {
	h := b.T - a.T
	if !(h > 0) { // rejects h <= 0 and NaN durations alike
		return StepRecord{}, ErrNonPositiveStep
	}
	i := s.nextStep

	raw := rk4Step(s.q, a, b, h)
	rawNorm := quaternion.Norm(raw)
	drift := math.Abs(rawNorm - 1.0)
	if drift > s.maxDrift {
		s.maxDrift = drift
	}

	if drift > s.threshold {
		if s.strict {
			return StepRecord{}, &DriftError{Step: i, Drift: drift, Threshold: s.threshold}
		}
		if len(s.warnings) < maxWarnings {
			s.warnings = append(s.warnings, "step "+itoa(i)+
				": quaternion norm drift "+formatFloat(drift)+
				" exceeded threshold "+formatFloat(s.threshold)+
				" (quaternion was renormalized; consider smaller steps)")
		}
	}

	nq, ok := quaternion.Normalized(raw)
	if !ok {
		return StepRecord{}, ErrZeroQuaternion
	}
	s.q = nq
	s.nextStep++

	wEnd := b.W
	speed := math.Sqrt(wEnd[0]*wEnd[0] + wEnd[1]*wEnd[1] + wEnd[2]*wEnd[2])
	return StepRecord{
		T:            b.T,
		Quaternion:   nq,
		NormBefore:   rawNorm,
		NormDrift:    drift,
		StepDuration: h,
		AngularSpeed: speed,
	}, nil
}

// AdvanceTo integrates from sample a up to time t strictly inside the interval
// (a.T, b.T). The angular velocity at t is taken from the SAME linear
// interpolation used between ordinary samples, and a synthetic endpoint sample
// is built from it, so the result equals a one-shot integration over a series
// truncated at t with one interpolated sample appended there.
func (s *Stepper) AdvanceTo(a, b Sample, t float64) (StepRecord, error) {
	if !(t > a.T) || t > b.T {
		return StepRecord{}, ErrNonPositiveStep
	}
	c := Sample{T: t, W: InterpolateRate(a, b, t)}
	return s.Advance(a, c)
}

// InterpolateRate returns the body angular velocity at time t obtained by
// linearly interpolating between the bracketing samples a and b.
func InterpolateRate(a, b Sample, t float64) [3]float64 {
	return omegaAt(a, b, t)
}

func cloneWarnings(w []string) []string {
	if len(w) == 0 {
		return nil
	}
	cp := make([]string, len(w))
	copy(cp, w)
	return cp
}
