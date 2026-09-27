package api_test

import (
	"bytes"
	"encoding/json"
	"fmt"
	"math"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"

	"github.com/example/attitude-service/internal/api"
	"github.com/example/attitude-service/internal/store"
)

func newRouter() http.Handler {
	return api.NewServer(store.New()).Router()
}

// doJSON is goroutine-safe: it never calls t.Fatal, errors come back as values.
func doJSON(r http.Handler, method, path string, body any) (int, map[string]any, error) {
	var buf bytes.Buffer
	if body != nil {
		if err := json.NewEncoder(&buf).Encode(body); err != nil {
			return 0, nil, fmt.Errorf("encode body: %w", err)
		}
	}
	req := httptest.NewRequest(method, path, &buf)
	req.Header.Set("Content-Type", "application/json")
	rec := httptest.NewRecorder()
	r.ServeHTTP(rec, req)
	out := map[string]any{}
	if rec.Body.Len() > 0 {
		if err := json.Unmarshal(rec.Body.Bytes(), &out); err != nil {
			return rec.Code, nil, fmt.Errorf("response is not JSON: %w (%s)", err, rec.Body.String())
		}
	}
	return rec.Code, out, nil
}

func mustJSON(t *testing.T, r http.Handler, method, path string, body any) (int, map[string]any) {
	t.Helper()
	code, out, err := doJSON(r, method, path, body)
	if err != nil {
		t.Fatalf("%s %s: %v", method, path, err)
	}
	return code, out
}

func yawSeriesPayload(rate float64, n int, dt float64) map[string]any {
	samples := make([]map[string]any, n)
	for i := 0; i < n; i++ {
		samples[i] = map[string]any{
			"t": float64(i) * dt,
			"w": []float64{0, 0, rate},
		}
	}
	return map[string]any{
		"q0":      []float64{1, 0, 0, 0},
		"samples": samples,
	}
}

// wrapAngle folds an angle in radians into (-π, π], matching the service's
// Euler-angle output range.
func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

func TestIntegrateHappyPath(t *testing.T) {
	r := newRouter()
	code, out := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate", yawSeriesPayload(0.5, 101, 0.1))
	if code != http.StatusOK {
		t.Fatalf("status = %d, out = %v", code, out)
	}
	// Terminal yaw must be 5 rad (rate 0.5 rad/s over 10 s), wrapped to (-π, π].
	ea := out["euler_angles"].(map[string]any)
	yaw := ea["yaw_rad"].(float64)
	want := wrapAngle(5.0)
	if math.Abs(yaw-want) > 1e-7 {
		t.Fatalf("yaw = %.10f, want %.10f", yaw, want)
	}
	if ea["singular"].(bool) {
		t.Fatal("pure yaw flagged singular")
	}
	// Convention must be stated in the response.
	conv := out["quaternion_convention"].(map[string]any)
	if conv["euler_order"] != "ZYX-intrinsic (yaw-pitch-roll)" {
		t.Fatalf("euler order = %v", conv["euler_order"])
	}
	if out["max_norm_drift"].(float64) >= out["norm_drift_threshold"].(float64) {
		t.Fatal("drift above threshold without warning")
	}
	if !out["normalized_after_each_step"].(bool) {
		t.Fatal("response must confirm per-step renormalization")
	}
}

func TestIntegrateRejectsBadInputs(t *testing.T) {
	r := newRouter()
	cases := []struct {
		name  string
		body  map[string]any
		cause string
	}{
		{"empty sequence", map[string]any{"q0": []float64{1, 0, 0, 0}, "samples": []any{}}, "empty_sequence"},
		{"non-positive step", map[string]any{
			"q0": []float64{1, 0, 0, 0},
			"samples": []map[string]any{
				{"t": 0.1, "w": []float64{0, 0, 1}},
				{"t": 0.1, "w": []float64{0, 0, 1}},
			},
		}, "non_positive_step"},
		{"non-unit q0", map[string]any{
			"q0": []float64{1, 1, 0, 0},
			"samples": []map[string]any{
				{"t": 0.0, "w": []float64{0, 0, 1}},
				{"t": 0.1, "w": []float64{0, 0, 1}},
			},
		}, "invalid_quaternion"},
		{"length mismatch", map[string]any{
			"q0":                 []float64{1, 0, 0, 0},
			"timestamps":         []float64{0, 0.1, 0.2},
			"angular_velocities": [][]float64{{0, 0, 1}, {0, 0, 1}},
		}, "length_mismatch"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			code, out := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate", tc.body)
			if code != http.StatusBadRequest {
				t.Fatalf("status = %d, want 400; out = %v", code, out)
			}
			if out["cause"] != tc.cause {
				t.Fatalf("cause = %v, want %s", out["cause"], tc.cause)
			}
			if out["detail"] == "" {
				t.Fatal("error response must carry a human-readable detail")
			}
		})
	}
}

func TestStrictDriftRejects(t *testing.T) {
	r := newRouter()
	body := yawSeriesPayload(0.5, 3, 0.1)
	body["max_step_norm_drift"] = 1e-18 // impossibly tight
	body["strict_drift"] = true
	code, out := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate", body)
	if code != http.StatusUnprocessableEntity {
		t.Fatalf("status = %d, want 422; out = %v", code, out)
	}
	if out["cause"] != "norm_drift_exceeded" {
		t.Fatalf("cause = %v", out["cause"])
	}
}

// TestNonStrictDriftWarnsAndContinues locks the two-mode drift policy: without
// strict mode an over-threshold step yields 200 with a non-empty warnings list.
func TestNonStrictDriftWarnsAndContinues(t *testing.T) {
	r := newRouter()
	body := yawSeriesPayload(0.5, 3, 0.1)
	body["max_step_norm_drift"] = 1e-18
	code, out := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate", body)
	if code != http.StatusOK {
		t.Fatalf("non-strict drift must not reject: status=%d out=%v", code, out)
	}
	w, ok := out["warnings"].([]any)
	if !ok || len(w) == 0 {
		t.Fatalf("expected drift warnings, got %v", out["warnings"])
	}
}

func TestSeriesSaveAndReuse(t *testing.T) {
	r := newRouter()
	// Save a named series: 0.8 rad/s over 1.0 s with fine steps.
	samples := make([]map[string]any, 51)
	for i := 0; i < 51; i++ {
		samples[i] = map[string]any{"t": float64(i) * 0.02, "w": []float64{0, 0, 0.8}}
	}
	code, _ := mustJSON(t, r, http.MethodPut, "/api/v1/series/spin-test",
		map[string]any{"samples": samples})
	if code != http.StatusCreated {
		t.Fatalf("save status = %d", code)
	}
	// Integrate by reference.
	code, out := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate",
		map[string]any{"q0": []float64{1, 0, 0, 0}, "series": "spin-test"})
	if code != http.StatusOK {
		t.Fatalf("integrate-by-name status = %d, out = %v", code, out)
	}
	ea := out["euler_angles"].(map[string]any)
	if math.Abs(ea["yaw_rad"].(float64)-0.8) > 1e-7 { // 0.8 rad/s * 1.0 s
		t.Fatalf("yaw = %v, want 0.8", ea["yaw_rad"])
	}
	// Unknown series name is a clean error.
	code, out = mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate",
		map[string]any{"q0": []float64{1, 0, 0, 0}, "series": "missing"})
	if code != http.StatusBadRequest || out["cause"] != "not_found" {
		t.Fatalf("missing series: status=%d out=%v", code, out)
	}
	// Delete and confirm gone.
	code, _ = mustJSON(t, r, http.MethodDelete, "/api/v1/series/spin-test", nil)
	if code != http.StatusOK {
		t.Fatalf("delete status = %d", code)
	}
	code, out = mustJSON(t, r, http.MethodGet, "/api/v1/series/spin-test", nil)
	if code != http.StatusNotFound || out["cause"] != "not_found" {
		t.Fatalf("deleted series still visible: status=%d", code)
	}
}

// TestConcurrentRunsAreIsolated drives two different integrations in parallel
// and requires each to produce exactly the same result as a serial run —
// no state may leak between concurrent integrations.
func TestConcurrentRunsAreIsolated(t *testing.T) {
	r := newRouter()

	serial := func(rate float64, n int, dt float64) map[string]any {
		code, out := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate", yawSeriesPayload(rate, n, dt))
		if code != http.StatusOK {
			t.Fatalf("serial run failed: %v", out)
		}
		return out
	}
	refA := serial(0.5, 101, 0.1)
	refB := serial(-1.3, 51, 0.2)

	run := func(payload map[string]any, ref map[string]any, tag string) error {
		code, out, err := doJSON(r, http.MethodPost, "/api/v1/attitude/integrate", payload)
		if err != nil {
			return fmt.Errorf("%s: %w", tag, err)
		}
		if code != http.StatusOK {
			return fmt.Errorf("%s: status %d", tag, code)
		}
		got := out["final_quaternion"].([]any)
		want := ref["final_quaternion"].([]any)
		for k := 0; k < 4; k++ {
			if got[k].(float64) != want[k].(float64) {
				return fmt.Errorf("%s: final quaternion deviated: %v vs %v", tag, got, want)
			}
		}
		return nil
	}

	var wg sync.WaitGroup
	errs := make(chan error, 200)
	for i := 0; i < 50; i++ {
		wg.Add(2)
		go func() {
			defer wg.Done()
			if err := run(yawSeriesPayload(0.5, 101, 0.1), refA, "run A"); err != nil {
				errs <- err
			}
		}()
		go func() {
			defer wg.Done()
			if err := run(yawSeriesPayload(-1.3, 51, 0.2), refB, "run B"); err != nil {
				errs <- err
			}
		}()
	}
	wg.Wait()
	close(errs)
	for err := range errs {
		t.Error(err)
	}
}
