// Package quaternion implements the basic quaternion algebra used by the
// attitude integrator.
//
// Convention (locked across the whole service):
//   - Components are stored scalar-first: q = [w, x, y, z], i.e. q = w + xi + yj + zk.
//   - Products follow Hamilton's convention: ij = k, jk = i, ki = j.
//   - A quaternion encodes the rotation from the body frame (b) to the
//     reference/inertial frame (n): v_n = q ⊗ v_b ⊗ q*.
//   - With that convention and the angular velocity expressed in the BODY
//     frame, the kinematic equation is q̇ = 1/2 · q ⊗ (0, ω_b) — the body
//     angular velocity quaternion is RIGHT-multiplied.
package quaternion

import "math"

// Q is a scalar-first Hamilton quaternion.
type Q struct {
	W float64
	X float64
	Y float64
	Z float64
}

// Identity returns the unit quaternion describing a zero rotation.
func Identity() Q {
	return Q{W: 1}
}

// Pure builds a zero-scalar quaternion from a 3-vector (used for angular rate).
func Pure(x, y, z float64) Q {
	return Q{X: x, Y: y, Z: z}
}

// Mul returns the Hamilton product a ⊗ b.
func Mul(a, b Q) Q {
	return Q{
		W: a.W*b.W - a.X*b.X - a.Y*b.Y - a.Z*b.Z,
		X: a.W*b.X + a.X*b.W + a.Y*b.Z - a.Z*b.Y,
		Y: a.W*b.Y - a.X*b.Z + a.Y*b.W + a.Z*b.X,
		Z: a.W*b.Z + a.X*b.Y - a.Y*b.X + a.Z*b.W,
	}
}

// Conj returns the quaternion conjugate q* (inverse rotation for unit q).
func Conj(q Q) Q {
	return Q{W: q.W, X: -q.X, Y: -q.Y, Z: -q.Z}
}

// Norm2 returns |q|².
func Norm2(q Q) float64 {
	return q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z
}

// Norm returns |q|.
func Norm(q Q) float64 {
	return math.Sqrt(Norm2(q))
}

// Normalized returns q/|q|. A zero quaternion has no direction and yields
// ok=false so callers never silently produce NaNs.
func Normalized(q Q) (n Q, ok bool) {
	m := Norm(q)
	if m == 0 || math.IsNaN(m) || math.IsInf(m, 0) {
		return Q{}, false
	}
	inv := 1.0 / m
	return Q{W: q.W * inv, X: q.X * inv, Y: q.Y * inv, Z: q.Z * inv}, true
}

// IsUnit reports whether q has unit norm within the given absolute tolerance.
func IsUnit(q Q, tol float64) bool {
	d := Norm(q) - 1.0
	if d < 0 {
		d = -d
	}
	return d <= tol
}

// IsFinite reports that none of the components is NaN or ±Inf.
func IsFinite(q Q) bool {
	comps := [4]float64{q.W, q.X, q.Y, q.Z}
	for _, c := range comps {
		if math.IsNaN(c) || math.IsInf(c, 0) {
			return false
		}
	}
	return true
}
