package euler_test

import (
	"math"
	"testing"

	"github.com/example/attitude-service/internal/euler"
	"github.com/example/attitude-service/internal/quaternion"
)

const tol = 1e-9

// qFromZYX builds the quaternion for yaw ψ, pitch θ, roll φ under the service's
// ZYX intrinsic convention (q = qz(ψ) ⊗ qy(θ) ⊗ qx(φ)), matching the body
// kinematic equation q̇ = ½q ⊗ ω_b.
func qFromZYX(yaw, pitch, roll float64) quaternion.Q {
	cz, sz := math.Cos(yaw/2), math.Sin(yaw/2)
	cy, sy := math.Cos(pitch/2), math.Sin(pitch/2)
	cr, sr := math.Cos(roll/2), math.Sin(roll/2)

	qz := quaternion.Q{W: cz, Z: sz}
	qy := quaternion.Q{W: cy, Y: sy}
	qx := quaternion.Q{W: cr, X: sr}
	return quaternion.Mul(quaternion.Mul(qz, qy), qx)
}

func TestIdentityAngles(t *testing.T) {
	a := euler.FromQuaternion(1, 0, 0, 0)
	if math.Abs(a.Roll)+math.Abs(a.Pitch)+math.Abs(a.Yaw) > tol {
		t.Fatalf("identity quaternion gave non-zero angles: %+v", a)
	}
	if a.Singular || a.NearSingular {
		t.Fatal("identity quaternion flagged singular")
	}
}

func TestZYXRoundTrip(t *testing.T) {
	cases := [][3]float64{
		{0.0, 0.0, 0.0},
		{1.0, 0.2, -0.4},
		{-2.3, 0.7, 0.3},
		{2.9, -0.6, -1.2},
		{0.5, 1.2, 2.1},
	}
	for _, c := range cases {
		yaw, pitch, roll := c[0], c[1], c[2]
		q := qFromZYX(yaw, pitch, roll)
		a := euler.FromQuaternion(q.W, q.X, q.Y, q.Z)
		if math.Abs(a.Yaw-yaw) > 1e-8 {
			t.Errorf("yaw mismatch: got %.9f want %.9f", a.Yaw, yaw)
		}
		if math.Abs(a.Pitch-pitch) > 1e-8 {
			t.Errorf("pitch mismatch: got %.9f want %.9f", a.Pitch, pitch)
		}
		if math.Abs(a.Roll-roll) > 1e-8 {
			t.Errorf("roll mismatch: got %.9f want %.9f", a.Roll, roll)
		}
		if a.Singular {
			t.Errorf("non-singular attitude flagged singular: %+v", a)
		}
	}
}

func TestGimbalLockIsFlaggedNotNaN(t *testing.T) {
	for _, pitch := range []float64{math.Pi / 2, -math.Pi / 2} {
		yaw, roll := 1.0, 0.5
		q := qFromZYX(yaw, pitch, roll)
		a := euler.FromQuaternion(q.W, q.X, q.Y, q.Z)
		if !a.Singular {
			t.Errorf("pitch %.0f not flagged singular", pitch*180/math.Pi)
		}
		if !a.NearSingular || a.Note == "" {
			t.Errorf("missing near-singular flag/note: %+v", a)
		}
		if math.IsNaN(a.Yaw) || math.IsNaN(a.Pitch) || math.IsNaN(a.Roll) {
			t.Errorf("NaN emitted in gimbal lock: %+v", a)
		}
		if a.Roll != 0 {
			t.Errorf("roll should be pinned to 0 at singularity, got %v", a.Roll)
		}
		// The single observable combination: yaw - roll (pitch +90) and
		// yaw + roll (pitch -90) must survive the fold into the pinned yaw.
		if pitch > 0 && math.Abs(a.Yaw-(yaw-roll)) > 1e-9 {
			t.Errorf("combined yaw at +90 pitch: got %.9f want %.9f", a.Yaw, yaw-roll)
		}
		if pitch < 0 && math.Abs(a.Yaw-(yaw+roll)) > 1e-9 {
			t.Errorf("combined yaw at -90 pitch: got %.9f want %.9f", a.Yaw, yaw+roll)
		}
		// Rebuilding the quaternion from the pinned angles must reproduce the
		// same attitude (q and -q are equivalent). Near exact gimbal lock the
		// extracted pitch loses a few digits to asin ill-conditioning, so a
		// tolerance looser than machine epsilon is appropriate.
		rebuilt := qFromZYX(a.Yaw, a.Pitch, a.Roll)
		d := math.Abs(rebuilt.W-q.W) + math.Abs(rebuilt.X-q.X) +
			math.Abs(rebuilt.Y-q.Y) + math.Abs(rebuilt.Z-q.Z)
		dNeg := math.Abs(rebuilt.W+q.W) + math.Abs(rebuilt.X+q.X) +
			math.Abs(rebuilt.Y+q.Y) + math.Abs(rebuilt.Z+q.Z)
		if math.Min(d, dNeg) > 1e-6 {
			t.Errorf("rebuilt attitude mismatch at pitch %.0f: %+v vs %+v",
				pitch*180/math.Pi, rebuilt, q)
		}
	}
}

func TestNearSingularFlag(t *testing.T) {
	q := qFromZYX(0.3, math.Pi/2-0.005, 0.1) // ~0.3° from the pole
	a := euler.FromQuaternion(q.W, q.X, q.Y, q.Z)
	if a.Singular || !a.NearSingular {
		t.Fatalf("expected near-singular but not singular: %+v", a)
	}
}

func TestConventionNameLocked(t *testing.T) {
	if euler.Order != "ZYX-intrinsic (yaw-pitch-roll)" {
		t.Fatalf("euler order convention changed: %s", euler.Order)
	}
}
