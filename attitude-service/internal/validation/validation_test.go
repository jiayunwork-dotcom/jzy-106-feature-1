package validation_test

import (
	"errors"
	"testing"

	"github.com/example/attitude-service/internal/validation"
)

func goodRates() []validation.AngularRate {
	return []validation.AngularRate{
		{T: 0.0, W: [3]float64{0, 0, 0.5}},
		{T: 0.1, W: [3]float64{0, 0, 0.5}},
		{T: 0.2, W: [3]float64{0, 0, 0.5}},
	}
}

func unitQ() [4]float64 { return [4]float64{1, 0, 0, 0} }

func expectCause(t *testing.T, err error, cause validation.Cause) {
	t.Helper()
	if err == nil {
		t.Fatalf("expected error with cause %s, got nil", cause)
	}
	var ve *validation.Error
	if !errors.As(err, &ve) {
		t.Fatalf("error %v is not a validation.Error", err)
	}
	if ve.Cause != cause {
		t.Fatalf("cause = %s, want %s (msg: %s)", ve.Cause, cause, ve.Message)
	}
	if ve.Message == "" {
		t.Fatal("validation error must carry a human-readable message")
	}
}

func TestRejectsEmptySequence(t *testing.T) {
	_, err := validation.ValidateIntegrate(unitQ(), nil, nil, false)
	expectCause(t, err, validation.CauseEmptySequence)
}

func TestRejectsNonPositiveStep(t *testing.T) {
	rates := goodRates()
	rates[1].T = rates[0].T // dt = 0
	_, err := validation.ValidateIntegrate(unitQ(), rates, nil, false)
	expectCause(t, err, validation.CauseNonPositiveStep)

	rates = goodRates()
	rates[2].T = 0.05 // dt < 0
	_, err = validation.ValidateIntegrate(unitQ(), rates, nil, false)
	expectCause(t, err, validation.CauseNonPositiveStep)
}

func TestRejectsNonUnitInitialQuaternion(t *testing.T) {
	_, err := validation.ValidateIntegrate([4]float64{2, 0, 0, 0}, goodRates(), nil, false)
	expectCause(t, err, validation.CauseInvalidQuaternion)

	_, err = validation.ValidateIntegrate([4]float64{0, 0, 0, 0}, goodRates(), nil, false)
	expectCause(t, err, validation.CauseInvalidQuaternion)
}

func TestAcceptsValidRequest(t *testing.T) {
	in, err := validation.ValidateIntegrate(unitQ(), goodRates(), nil, false)
	if err != nil {
		t.Fatalf("valid request rejected: %v", err)
	}
	if len(in.Samples) != 3 {
		t.Fatalf("samples = %d, want 3", len(in.Samples))
	}
	if in.MaxStepNormDrift <= 0 {
		t.Fatal("default drift threshold must be positive")
	}
}

func TestRejectsBadThreshold(t *testing.T) {
	neg := -1.0
	_, err := validation.ValidateIntegrate(unitQ(), goodRates(), &neg, false)
	expectCause(t, err, validation.CauseInvalidThreshold)
	zero := 0.0
	_, err = validation.ValidateIntegrate(unitQ(), goodRates(), &zero, false)
	expectCause(t, err, validation.CauseInvalidThreshold)
}

func TestValidateSeries(t *testing.T) {
	if _, err := validation.ValidateSeries("spin-test", goodRates()); err != nil {
		t.Fatalf("valid series rejected: %v", err)
	}
	_, err := validation.ValidateSeries("bad name with spaces", goodRates())
	expectCause(t, err, validation.CauseInvalidName)
	_, err = validation.ValidateSeries("ok-name", nil)
	expectCause(t, err, validation.CauseEmptySequence)
}
