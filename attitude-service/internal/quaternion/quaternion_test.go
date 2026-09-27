package quaternion_test

import (
	"math"
	"testing"

	"github.com/example/attitude-service/internal/quaternion"
)

func approxEq(a, b, tol float64) bool { return math.Abs(a-b) <= tol }

func approxEqQ(a, b quaternion.Q, tol float64) bool {
	return approxEq(a.W, b.W, tol) && approxEq(a.X, b.X, tol) &&
		approxEq(a.Y, b.Y, tol) && approxEq(a.Z, b.Z, tol)
}

func TestHamiltonProducts(t *testing.T) {
	i := quaternion.Q{X: 1}
	j := quaternion.Q{Y: 1}
	k := quaternion.Q{Z: 1}
	if got := quaternion.Mul(i, j); !approxEqQ(got, k, 1e-15) {
		t.Fatalf("ij = %+v, want k", got)
	}
	if got := quaternion.Mul(j, k); !approxEqQ(got, i, 1e-15) {
		t.Fatalf("jk = %+v, want i", got)
	}
	if got := quaternion.Mul(k, i); !approxEqQ(got, j, 1e-15) {
		t.Fatalf("ki = %+v, want j", got)
	}
	// Anti-commutativity of the basis vectors: ji = -k.
	if got := quaternion.Mul(j, i); !approxEqQ(got, quaternion.Q{Z: -1}, 1e-15) {
		t.Fatalf("ji = %+v, want -k", got)
	}
}

func TestConjIsInverseForUnitQuaternion(t *testing.T) {
	q := must(quaternion.Q{W: 0.3, X: -2, Y: 1.4, Z: 0.9})
	inv := quaternion.Conj(q)
	if got := quaternion.Mul(q, inv); !approxEqQ(got, quaternion.Identity(), 1e-15) {
		t.Fatalf("q q* = %+v, want identity", got)
	}
}

func must(q quaternion.Q) quaternion.Q {
	n, ok := quaternion.Normalized(q)
	if !ok {
		panic("normalize failed")
	}
	return n
}

func TestNormalizeAndIsUnit(t *testing.T) {
	q := quaternion.Q{W: 1, X: 2, Y: 3, Z: 4}
	n, ok := quaternion.Normalized(q)
	if !ok {
		t.Fatal("normalize failed")
	}
	if !approxEq(quaternion.Norm(n), 1, 1e-15) {
		t.Fatalf("normalized norm = %v", quaternion.Norm(n))
	}
	if !quaternion.IsUnit(n, 1e-12) {
		t.Fatal("normalized quaternion reported non-unit")
	}
	if quaternion.IsUnit(q, 1e-9) {
		t.Fatal("non-unit quaternion reported unit")
	}
	if _, ok := quaternion.Normalized(quaternion.Q{}); ok {
		t.Fatal("zero quaternion should not normalize")
	}
}

func TestMulIsNotCommutative(t *testing.T) {
	a := quaternion.Q{W: 1, X: 2, Y: 3, Z: 4}
	b := quaternion.Q{W: 0.4, X: -1, Y: 2.2, Z: 0.7}
	if approxEqQ(quaternion.Mul(a, b), quaternion.Mul(b, a), 1e-12) {
		t.Fatal("Hamilton product unexpectedly commutative")
	}
}
