// Package integrator advances an attitude quaternion along a sampled
// body-frame angular velocity time series using a classical fourth-order
// Runge–Kutta scheme on the quaternion kinematic equation
//
//	q̇ = 1/2 · q ⊗ (0, ω_b),
//
// i.e. the body angular velocity is RIGHT-multiplied (see package quaternion).
// The quaternion is renormalized after every completed step so that its norm
// stays exactly one regardless of accumulated round-off.
package integrator

import (
	"errors"
	"strconv"

	"github.com/example/attitude-service/internal/quaternion"
)

// Sample is one angular-velocity observation, in rad/s expressed in the body
// frame, together with the timestamp at which it is observed, in seconds.
type Sample struct {
	T float64
	W [3]float64
}

// StepRecord holds the post-step attitude and diagnostics for one integration
// step (between sample i-1 and sample i).
type StepRecord struct {
	T            float64
	Quaternion   quaternion.Q
	NormBefore   float64 // |q| produced by RK4, before renormalization
	NormDrift    float64 // ||q|-1| of the raw RK4 result
	StepDuration float64
	AngularSpeed float64 // |ω| (interpolated at the end of the step)
}

// Options controls integration diagnostics.
type Options struct {
	// MaxStepNormDrift is the largest allowed per-step ||q|-1| drift before
	// renormalization. Zero selects the built-in default.
	MaxStepNormDrift float64
	// StrictDrift rejects the run with ErrDriftThreshold as soon as the drift
	// threshold is exceeded. When false, threshold breaches are reported as
	// warnings and integration continues (the quaternion is still normalized).
	StrictDrift bool
}

// Result is the full outcome of one integration run. All intermediate state
// lives in a single Result value, so two runs can never share state.
type Result struct {
	Initial     quaternion.Q
	Final       quaternion.Q
	Steps       []StepRecord
	ElapsedTime float64
	MaxDrift    float64
	Warnings    []string
}

// DefaultMaxStepNormDrift is used when no explicit threshold is supplied.
const DefaultMaxStepNormDrift = 1e-4

const maxWarnings = 20

// Errors returned by Simulate.
var (
	ErrNoSamples       = errors.New("angular velocity series is empty")
	ErrNonPositiveStep = errors.New("time step is non-positive")
	ErrZeroQuaternion  = errors.New("quaternion collapsed to zero, cannot renormalize")
	ErrDriftThreshold  = errors.New("single-step norm drift exceeded threshold")
)

// DriftError wraps ErrDriftThreshold with the offending step's diagnostics.
type DriftError struct {
	Step      int
	Drift     float64
	Threshold float64
}

func (e *DriftError) Error() string {
	return "step " + itoa(e.Step) + ": quaternion norm drift " +
		formatFloat(e.Drift) + " exceeded threshold " + formatFloat(e.Threshold)
}

// Is enables errors.Is(err, ErrDriftThreshold).
func (e *DriftError) Is(target error) bool { return target == ErrDriftThreshold }

// omegaAt linearly interpolates the body angular velocity at time t between
// the two bracketing samples. Samples with duration 0 are guarded by the
// caller, so division by zero cannot occur here.
func omegaAt(a, b Sample, t float64) [3]float64 {
	u := (t - a.T) / (b.T - a.T)
	var w [3]float64
	for i := 0; i < 3; i++ {
		w[i] = a.W[i] + u*(b.W[i]-a.W[i])
	}
	return w
}

// qdot evaluates the quaternion rate q̇ = 1/2 q ⊗ (0, ω_b).
func qdot(q quaternion.Q, w [3]float64) quaternion.Q {
	half := 0.5
	return quaternion.Q{
		W: half * (-q.X*w[0] - q.Y*w[1] - q.Z*w[2]),
		X: half * (q.W*w[0] + q.Y*w[2] - q.Z*w[1]),
		Y: half * (q.W*w[1] - q.X*w[2] + q.Z*w[0]),
		Z: half * (q.W*w[2] + q.X*w[1] - q.Y*w[0]),
	}
}

func addScaled(q quaternion.Q, d quaternion.Q, h float64) quaternion.Q {
	return quaternion.Q{
		W: q.W + h*d.W,
		X: q.X + h*d.X,
		Y: q.Y + h*d.Y,
		Z: q.Z + h*d.Z,
	}
}

// rk4Step performs one classical RK4 step of duration h, interpolating the
// angular velocity linearly between the bracketing samples. No normalization
// is applied between the internal stages.
func rk4Step(q quaternion.Q, a, b Sample, h float64) quaternion.Q {
	tMid := a.T + 0.5*h
	w0 := a.W
	w1 := omegaAt(a, b, tMid)
	w2 := w1
	w3 := b.W

	k1 := qdot(q, w0)
	k2 := qdot(addScaled(q, k1, 0.5*h), w1)
	k3 := qdot(addScaled(q, k2, 0.5*h), w2)
	k4 := qdot(addScaled(q, k3, h), w3)

	sixth := h / 6.0
	return quaternion.Q{
		W: q.W + sixth*(k1.W+2*k2.W+2*k3.W+k4.W),
		X: q.X + sixth*(k1.X+2*k2.X+2*k3.X+k4.X),
		Y: q.Y + sixth*(k1.Y+2*k2.Y+2*k3.Y+k4.Y),
		Z: q.Z + sixth*(k1.Z+2*k2.Z+2*k3.Z+k4.Z),
	}
}

// Simulate integrates the attitude from q0 along samples. Validation of the
// input (lengths, non-positive steps, unit initial quaternion) is the
// validation package's responsibility; this function assumes well-formed
// inputs but still guards every step duration.
//
// The whole run is executed through the incremental Stepper; nothing here
// duplicates the step math. This guarantees that a stateful trajectory
// appending the same samples reaches a bit-for-bit identical final quaternion.
func Simulate(q0 quaternion.Q, samples []Sample, opts Options) (*Result, error) {
	if len(samples) == 0 {
		return nil, ErrNoSamples
	}

	st := NewStepper(q0, opts)
	res := &Result{
		Initial: q0,
		Final:   q0,
		Steps:   make([]StepRecord, 0, len(samples)),
	}

	for i := 1; i < len(samples); i++ {
		rec, err := st.Advance(samples[i-1], samples[i])
		if err != nil {
			return nil, err
		}
		res.Steps = append(res.Steps, rec)
	}

	res.Final = st.Attitude()
	res.MaxDrift = st.MaxDrift()
	res.Warnings = st.Warnings()
	res.ElapsedTime = samples[len(samples)-1].T - samples[0].T
	return res, nil
}

// itoa is a tiny dependency-free int-to-string for diagnostic messages.
func itoa(i int) string {
	if i == 0 {
		return "0"
	}
	var buf [24]byte
	pos := len(buf)
	for i > 0 {
		pos--
		buf[pos] = byte('0' + i%10)
		i /= 10
	}
	return string(buf[pos:])
}

// formatFloat renders a float with scientific precision for diagnostic messages.
func formatFloat(f float64) string {
	return strconv.FormatFloat(f, 'g', -1, 64)
}
