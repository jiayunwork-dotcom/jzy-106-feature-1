package integrator_test

import (
	"testing"

	"github.com/example/attitude-service/internal/integrator"
	"github.com/example/attitude-service/internal/quaternion"
)

// TestLargeStepWarnsButContinues verifies the non-strict policy: a step whose
// raw norm drift exceeds the threshold is reported as a warning while the run
// still completes and returns a unit-norm attitude.
func TestLargeStepWarnsButContinues(t *testing.T) {
	samples := []integrator.Sample{
		{T: 0, W: [3]float64{0, 0, 0.5}},
		{T: 1.0, W: [3]float64{0, 0, 0.5}}, // one coarse step; raw drift ~O(h⁴)
	}
	res, err := integrator.Simulate(quaternion.Identity(), samples, integrator.Options{
		MaxStepNormDrift: 1e-12,
		StrictDrift:      false,
	})
	if err != nil {
		t.Fatalf("non-strict run must not error: %v", err)
	}
	if len(res.Warnings) != 1 {
		t.Fatalf("warnings = %v, want exactly one", res.Warnings)
	}
	if n := quaternion.Norm(res.Final); n < 0.999999999999 || n > 1.000000000001 {
		t.Fatalf("final norm after renormalization = %v", n)
	}
	if res.MaxDrift <= 1e-12 {
		t.Fatal("MaxDrift should record the breach")
	}
}

// TestLargeStepStrictRejects verifies the strict policy: the same coarse step
// with StrictDrift aborts before returning an attitude.
func TestLargeStepStrictRejects(t *testing.T) {
	samples := []integrator.Sample{
		{T: 0, W: [3]float64{0, 0, 0.5}},
		{T: 1.0, W: [3]float64{0, 0, 0.5}},
	}
	_, err := integrator.Simulate(quaternion.Identity(), samples, integrator.Options{
		MaxStepNormDrift: 1e-12,
		StrictDrift:      true,
	})
	if err == nil {
		t.Fatal("strict run must reject when drift exceeds threshold")
	}
}

func TestSingleSampleIsZeroSteps(t *testing.T) {
	res, err := integrator.Simulate(quaternion.Identity(),
		[]integrator.Sample{{T: 2.5, W: [3]float64{1, 2, 3}}}, integrator.Options{})
	if err != nil {
		t.Fatalf("single sample should integrate trivially: %v", err)
	}
	if len(res.Steps) != 0 || res.ElapsedTime != 0 {
		t.Fatalf("single sample produced steps: %+v", res)
	}
}
