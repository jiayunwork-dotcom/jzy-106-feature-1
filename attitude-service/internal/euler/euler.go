// Package euler converts attitude quaternions to Euler angles.
//
// The rotation order convention is fixed for the whole service and is always
// reported in the response:
//
//	ZYX intrinsic, i.e. yaw (ψ) about reference z, then pitch (θ) about the
//	new y, then roll (φ) about the newest x — the aerospace yaw/pitch/roll
//	convention. All angles are in radians in [-π, π] (pitch in [-π/2, π/2]).
//
// Near |pitch| = π/2 the ZYX decomposition is singular: yaw and roll become
// individually unobservable (only a sum/difference of them is determined by
// the attitude). Instead of emitting a misleading number — or NaN from a
// division by zero — the conversion flags the singularity, pins roll to zero,
// and folds the combined angle into yaw.
package euler

import "math"

// Order is the fixed Euler convention identifier returned to every client.
const Order = "ZYX-intrinsic (yaw-pitch-roll)"

// Tolerances on cos(pitch). Below SingularCosTol the attitude is treated as
// exactly gimbal-locked; below NearSingularCosTol (~1 degree from the poles)
// a near-singularity warning is raised while normal extraction still applies.
const (
	SingularCosTol     = 1e-6
	NearSingularCosTol = 0.0174524064372835 // sin(1°)
)

// Angles is the Euler decomposition of an attitude quaternion.
type Angles struct {
	Roll         float64 // φ, radians
	Pitch        float64 // θ, radians
	Yaw          float64 // ψ, radians
	Singular     bool    // exactly at (or numerically on) the gimbal-lock pole
	NearSingular bool    // close enough that yaw/roll separation is ill-conditioned
	Note         string  // human-readable singularity explanation, empty otherwise
}

// FromQuaternion extracts ZYX intrinsic Euler angles from q using its
// equivalent rotation matrix R (v_n = R v_b):
//
//	R = | 1-2(y²+z²)   2(xy-wz)    2(xz+wy)  |
//	    |  2(xy+wz)    1-2(x²+z²)   2(yz-wx)  |
//	    |  2(xz-wy)     2(yz+wx)   1-2(x²+y²) |
//
// θ = asin(-R20), ψ = atan2(R10, R00), φ = atan2(R21, R22).
func FromQuaternion(w, x, y, z float64) Angles {
	r00 := 1 - 2*(y*y+z*z)
	r01 := 2 * (x*y - w*z)
	r10 := 2 * (x*y + w*z)
	r11 := 1 - 2*(x*x+z*z)
	r20 := 2 * (x*z - w*y)
	r21 := 2 * (y*z + w*x)
	r22 := 1 - 2*(x*x+y*y)

	sinPitch := clamp(r20*-1, -1, 1)
	pitch := math.Asin(sinPitch)
	cosPitch := math.Sqrt(math.Max(0, 1-sinPitch*sinPitch))

	a := Angles{Pitch: pitch}

	switch {
	case cosPitch < SingularCosTol:
		// Gimbal locked. Fold the single observable combination into yaw,
		// pin roll to zero. The pinned value is chosen so that rebuilding the
		// quaternion from the returned angles reproduces the same attitude.
		a.Singular = true
		a.NearSingular = true
		if sinPitch > 0 {
			// pitch = +90°: only (roll - yaw) is observable; with roll pinned
			// to 0, yaw_eff = yaw - roll = atan2(-R01, R11).
			a.Yaw = normAngle(math.Atan2(-r01, r11))
			a.Note = "gimbal lock at pitch +90°: yaw and roll are not individually " +
				"observable; roll pinned to 0 and yaw holds the combined angle"
		} else {
			// pitch = -90°: only (roll + yaw) is observable; with roll pinned
			// to 0, yaw_eff = yaw + roll = atan2(-R01, R11).
			a.Yaw = normAngle(math.Atan2(-r01, r11))
			a.Note = "gimbal lock at pitch -90°: yaw and roll are not individually " +
				"observable; roll pinned to 0 and yaw holds the combined angle"
		}
	case cosPitch < NearSingularCosTol:
		a.NearSingular = true
		a.Yaw = normAngle(math.Atan2(r10, r00))
		a.Roll = normAngle(math.Atan2(r21, r22))
		a.Note = "attitude is within ~1° of the pitch ±90° singularity; " +
			"yaw/roll separation is ill-conditioned"
	default:
		a.Yaw = normAngle(math.Atan2(r10, r00))
		a.Roll = normAngle(math.Atan2(r21, r22))
	}
	return a
}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// normAngle maps an angle in radians into (-π, π].
func normAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}
