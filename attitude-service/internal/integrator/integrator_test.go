package integrator_test

import (
	"math"
	"testing"

	"github.com/example/attitude-service/internal/euler"
	"github.com/example/attitude-service/internal/integrator"
	"github.com/example/attitude-service/internal/quaternion"
)

const invTol = 1e-10

// rateFn is an arbitrary time-varying, multi-axis body angular velocity used
// to make the checks genuinely non-trivial (units: rad/s).
func rateFn(t float64) [3]float64 {
	return [3]float64{
		0.7*math.Sin(1.3*t) + 0.2,
		-0.5*math.Cos(0.9*t) + 0.1*t,
		0.4*math.Sin(0.5*t+0.6) - 0.3,
	}
}

func makeSeries(n int, t0, t1 float64, scaleW, scaleT float64, negate bool) []integrator.Sample {
	s := make([]integrator.Sample, n)
	for i := 0; i < n; i++ {
		u := float64(i) / float64(n-1)
		tRaw := t0 + u*(t1-t0)
		w := rateFn(tRaw)
		for k := range w {
			w[k] *= scaleW
			if negate {
				w[k] = -w[k]
			}
		}
		s[i] = integrator.Sample{T: tRaw * scaleT, W: w}
	}
	return s
}

// quatsEqualUpToSign compares two quaternions as attitudes: q and -q describe
// the same orientation.
func quatsEqualUpToSign(a, b quaternion.Q, tol float64) bool {
	d := func(sign float64) float64 {
		return math.Abs(a.W-sign*b.W) + math.Abs(a.X-sign*b.X) +
			math.Abs(a.Y-sign*b.Y) + math.Abs(a.Z-sign*b.Z)
	}
	return math.Min(d(1), d(-1)) <= tol
}

// TestDoubleRateHalfStepSameAttitude locks: doubling every angular velocity and
// halving every time step leaves the terminal attitude unchanged, because the
// integrated rotation angle ½∫|ω|dt per axis is the same.
func TestDoubleRateHalfStepSameAttitude(t *testing.T) {
	base := makeSeries(120, 0, 6.0, 1, 1, false)
	doubled := makeSeries(120, 0, 6.0, 2.0, 0.5, false)

	r1, err := integrator.Simulate(quaternion.Identity(), base, integrator.Options{})
	if err != nil {
		t.Fatalf("base integration: %v", err)
	}
	r2, err := integrator.Simulate(quaternion.Identity(), doubled, integrator.Options{})
	if err != nil {
		t.Fatalf("scaled integration: %v", err)
	}
	if !quatsEqualUpToSign(r1.Final, r2.Final, invTol) {
		t.Fatalf("double-rate/half-step changed attitude:\n base    = %+v\n scaled  = %+v",
			r1.Final, r2.Final)
	}
}

// TestNegatedRateIsReversedRotation locks: negating the whole angular velocity
// series produces the inverse terminal rotation of the original process
// (verified for one fixed rotation axis, where the end orientations coincide).
func TestNegatedRateIsReversedRotation(t *testing.T) {
	// Fixed-axis series: rotating -ω over the same schedule is the inverse
	// rotation of the +ω run, i.e. final(-ω) = final(ω)^{-1} = conj(final(ω)).
	const n = 60
	fwd := make([]integrator.Sample, n)
	neg := make([]integrator.Sample, n)
	axis := [3]float64{0.6, -0.8, 0.27} // |axis| ≈ 1.025
	for i := 0; i < n; i++ {
		t := float64(i) * 0.05
		var w [3]float64
		for k := range w {
			w[k] = (0.8 + 0.3*math.Sin(t)) * axis[k]
		}
		fwd[i] = integrator.Sample{T: t, W: w}
		for k := range w {
			w[k] = -w[k]
		}
		neg[i] = integrator.Sample{T: t, W: w}
	}

	rf, err := integrator.Simulate(quaternion.Identity(), fwd, integrator.Options{})
	if err != nil {
		t.Fatalf("forward integration: %v", err)
	}
	rn, err := integrator.Simulate(quaternion.Identity(), neg, integrator.Options{})
	if err != nil {
		t.Fatalf("negated integration: %v", err)
	}
	want := quaternion.Conj(rf.Final)
	if !quatsEqualUpToSign(want, rn.Final, invTol) {
		t.Fatalf("negated rates are not the inverse rotation:\n forward = %+v\n neg     = %+v\n want    = %+v",
			rf.Final, rn.Final, want)
	}

	// General kinematic check for an arbitrary multi-axis, time-varying
	// series: integrating forward with +ω and then back with -ω over the same
	// reversed time grid must return (up to numerical error) to the starting
	// attitude.
	nr := 400
	span := 4.0
	fwd2 := makeSeries(nr, 0, span, 1, 1, false)
	rev := make([]integrator.Sample, nr)
	dt := span / float64(nr-1)
	for i := 0; i < nr; i++ {
		src := fwd2[nr-1-i]
		rev[i] = integrator.Sample{T: float64(i) * dt, W: neg3(src.W)}
	}
	rf2, err := integrator.Simulate(quaternion.Identity(), fwd2, integrator.Options{})
	if err != nil {
		t.Fatalf("forward leg: %v", err)
	}
	rc, err := integrator.Simulate(rf2.Final, rev, integrator.Options{})
	if err != nil {
		t.Fatalf("reverse leg: %v", err)
	}
	if !quatsEqualUpToSign(quaternion.Identity(), rc.Final, 1e-7) {
		t.Fatalf("forward-then-reverse did not close the loop: final=%+v", rc.Final)
	}
}

func neg3(v [3]float64) [3]float64 { return [3]float64{-v[0], -v[1], -v[2]} }

// TestFullTurnIsSameAttitude locks: a full revolution about one fixed axis
// returns the initial attitude (quaternion may flip its overall sign).
func TestFullTurnIsSameAttitude(t *testing.T) {
	axis := normalize3([3]float64{1, 2, -3})
	speed := 0.9 // rad/s
	n := 301
	s := make([]integrator.Sample, n)
	for i := 0; i < n; i++ {
		t := float64(i) * (2 * math.Pi / speed) / float64(n-1)
		s[i] = integrator.Sample{
			T: t,
			W: [3]float64{speed * axis[0], speed * axis[1], speed * axis[2]},
		}
	}
	r, err := integrator.Simulate(quaternion.Identity(), s, integrator.Options{})
	if err != nil {
		t.Fatalf("integration: %v", err)
	}
	if !quatsEqualUpToSign(quaternion.Identity(), r.Final, 1e-9) {
		t.Fatalf("full revolution did not return to the initial attitude: %+v", r.Final)
	}
}

// TestZeroRateFreezesAttitude locks: with ω ≡ 0 the attitude never moves.
func TestZeroRateFreezesAttitude(t *testing.T) {
	q0 := quaternion.Q{W: 0.5, X: 0.5, Y: 0.5, Z: 0.5}
	n := 30
	s := make([]integrator.Sample, n)
	for i := 0; i < n; i++ {
		s[i] = integrator.Sample{T: float64(i) * 0.1}
	}
	r, err := integrator.Simulate(q0, s, integrator.Options{})
	if err != nil {
		t.Fatalf("integration: %v", err)
	}
	if !quatsEqualUpToSign(q0, r.Final, 1e-12) {
		t.Fatalf("attitude drifted under zero angular velocity: %+v", r.Final)
	}
	for i, st := range r.Steps {
		if math.Abs(quaternion.Norm(st.Quaternion)-1) > 1e-12 {
			t.Fatalf("step %d not unit norm: %g", i, quaternion.Norm(st.Quaternion))
		}
	}
}

// TestConstantAxisRotationAngle locks: constant ω about a fixed axis rotates
// through angle |ω|·t, with the axis preserved.
func TestConstantAxisRotationAngle(t *testing.T) {
	axis := normalize3([3]float64{-2.0, 1.5, 4.0})
	speed := 1.7 // rad/s
	elapsed := 1.0
	n := 21 // h = 0.05 s -> h·|ω| ≈ 0.085 rad, well inside the step-drift limit
	s := make([]integrator.Sample, n)
	for i := 0; i < n; i++ {
		tt := float64(i) / float64(n-1) * elapsed
		s[i] = integrator.Sample{
			T: tt,
			W: [3]float64{speed * axis[0], speed * axis[1], speed * axis[2]},
		}
	}
	r, err := integrator.Simulate(quaternion.Identity(), s, integrator.Options{})
	if err != nil {
		t.Fatalf("integration: %v", err)
	}

	// Rotation angle from quaternion: θ = 2·acos(w) (fold to [0, π]).
	cosHalf := math.Abs(r.Final.W)
	angle := 2 * math.Acos(math.Max(-1, math.Min(1, cosHalf)))
	want := speed * elapsed
	if math.Abs(angle-want) > 1e-6 {
		t.Fatalf("swept angle mismatch: got %.10f want %.10f", angle, want)
	}

	// Rotation axis must equal the commanded axis up to sign.
	sinHalf := math.Sqrt(math.Max(0, 1-cosHalf*cosHalf))
	gotAxis := [3]float64{
		r.Final.X / sinHalf, r.Final.Y / sinHalf, r.Final.Z / sinHalf,
	}
	if r.Final.W < 0 {
		for k := range gotAxis {
			gotAxis[k] = -gotAxis[k]
		}
	}
	d := math.Abs(gotAxis[0]-axis[0]) + math.Abs(gotAxis[1]-axis[1]) + math.Abs(gotAxis[2]-axis[2])
	if d > 1e-6 {
		t.Fatalf("rotation axis mismatch: got %v want %v", gotAxis, axis)
	}
}

// TestPureYawBenchmark is the hand-checkable baseline: pure rotation about the
// body vertical (z) axis must yield yaw growing linearly with time while pitch
// and roll stay ~zero.
func TestPureYawBenchmark(t *testing.T) {
	yawRate := 0.5 // rad/s about z
	elapsed := 10.0
	n := 1001 // dt = 0.01 s: RK4 truncation stays far below 1e-9 on the terminal value
	s := make([]integrator.Sample, n)
	for i := 0; i < n; i++ {
		tt := float64(i) / float64(n-1) * elapsed
		s[i] = integrator.Sample{T: tt, W: [3]float64{0, 0, yawRate}}
	}
	r, err := integrator.Simulate(quaternion.Identity(), s, integrator.Options{})
	if err != nil {
		t.Fatalf("integration: %v", err)
	}
	for i, st := range r.Steps {
		ea := euler.FromQuaternion(st.Quaternion.W, st.Quaternion.X, st.Quaternion.Y, st.Quaternion.Z)
		wantYaw := normAngle(yawRate * st.T)
		if math.Abs(ea.Yaw-wantYaw) > 1e-7 {
			t.Fatalf("step %d t=%.2f: yaw %.8f != linear %.8f", i, st.T, ea.Yaw, wantYaw)
		}
		if math.Abs(ea.Pitch) > 1e-10 {
			t.Fatalf("step %d: pitch %.3e, want ~0", i, ea.Pitch)
		}
		if math.Abs(ea.Roll) > 1e-10 {
			t.Fatalf("step %d: roll %.3e, want ~0", i, ea.Roll)
		}
	}
	// Terminal hand calculation: yaw = 5 rad.
	ea := euler.FromQuaternion(r.Final.W, r.Final.X, r.Final.Y, r.Final.Z)
	if math.Abs(ea.Yaw-normAngle(yawRate*elapsed)) > 1e-9 {
		t.Fatalf("terminal yaw %.10f, want %.10f", ea.Yaw, normAngle(yawRate*elapsed))
	}
}

func normalize3(v [3]float64) [3]float64 {
	m := math.Sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
	return [3]float64{v[0] / m, v[1] / m, v[2] / m}
}

func normAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}
