package api_test

import (
	"encoding/json"
	"fmt"
	"math"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"testing"
)

// openTraj creates a trajectory via HTTP and returns its id.
func openTraj(t *testing.T, r http.Handler, body map[string]any) string {
	t.Helper()
	if body == nil {
		body = map[string]any{"q0": []float64{1, 0, 0, 0}}
	}
	code, out := mustJSON(t, r, http.MethodPost, "/api/v1/trajectories", body)
	if code != http.StatusCreated {
		t.Fatalf("create status=%d out=%v", code, out)
	}
	return out["id"].(string)
}

func packetBody(seq int64, samples []map[string]any, ver *int64) map[string]any {
	b := map[string]any{"seq": seq, "samples": samples}
	if ver != nil {
		b["expected_version"] = *ver
	}
	return b
}

func yawSamples(rate float64, i0, i1 int, dt float64) []map[string]any {
	out := make([]map[string]any, i1-i0)
	for i := i0; i < i1; i++ {
		out[i-i0] = map[string]any{"t": float64(i) * dt, "w": []float64{0, 0, rate}}
	}
	return out
}

// tipResult unwraps the append response's embedded "result" attitude payload.
func tipResult(t *testing.T, out map[string]any) map[string]any {
	t.Helper()
	res, ok := out["result"].(map[string]any)
	if !ok {
		t.Fatalf("append response has no result: %v", out)
	}
	return res
}

// TestTrajectoryHTTPEndToEnd drives a multi-packet append and checks the tip
// against the one-shot endpoint over the same samples.
func TestTrajectoryHTTPEndToEnd(t *testing.T) {
	r := newRouter()
	id := openTraj(t, r, nil)

	// 101 samples t=0..10s, 0.5 rad/s yaw, delivered in uneven packets.
	bounds := []int{0, 1, 5, 40, 101}
	var ver int64
	for k := 1; k < len(bounds); k++ {
		code, out := mustJSON(t, r, http.MethodPost,
			fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
			packetBody(int64(k), yawSamples(0.5, bounds[k-1], bounds[k], 0.1), &ver))
		if code != http.StatusOK {
			t.Fatalf("packet %d status=%d out=%v", k, code, out)
		}
		ver = int64(out["version"].(float64))
		if out["duplicate"].(bool) {
			t.Fatalf("packet %d marked duplicate", k)
		}
		if k == 1 {
			// First packet with one sample integrates zero steps.
			if out["recomputed_steps"].(float64) != 0 {
				t.Fatalf("first packet recomputed %v steps", out["recomputed_steps"])
			}
		}
	}

	// Compare against the one-shot endpoint.
	code, one := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate",
		yawSeriesPayload(0.5, 101, 0.1))
	if code != http.StatusOK {
		t.Fatalf("one-shot: %d", code)
	}
	code, got := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s", id), nil)
	if code != http.StatusOK {
		t.Fatalf("get: %d %v", code, got)
	}
	gq := got["final_quaternion"].([]any)
	oq := one["final_quaternion"].([]any)
	for i := range oq {
		if gq[i].(float64) != oq[i].(float64) {
			t.Fatalf("tip quaternion %v != one-shot %v", gq, oq)
		}
	}
	if got["max_norm_drift"].(float64) != one["max_norm_drift"].(float64) {
		t.Fatal("max drift differs")
	}
	// Euler singularity flags present.
	ea := got["euler_angles"].(map[string]any)
	if ea["singular"].(bool) {
		t.Fatal("pure yaw flagged singular")
	}
	if yaw := ea["yaw_rad"].(float64); math.Abs(yaw-wrapAngle(5.0)) > 1e-7 {
		t.Fatalf("yaw %v", yaw)
	}
}

// TestTrajectoryHTTPRetransmit verifies idempotency over HTTP.
func TestTrajectoryHTTPRetransmit(t *testing.T) {
	r := newRouter()
	id := openTraj(t, r, nil)
	p1 := packetBody(1, yawSamples(0.5, 0, 20, 0.1), nil)
	_, out := mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id), p1)
	v := out["version"].(float64)
	q1 := tipResult(t, out)["final_quaternion"].([]any)

	// Identical retransmit: duplicate, version unchanged.
	code, out2 := mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(1, yawSamples(0.5, 0, 20, 0.1), nil))
	if code != http.StatusOK || !out2["duplicate"].(bool) {
		t.Fatalf("retransmit: %d %v", code, out2)
	}
	if out2["version"].(float64) != v || out2["recomputed_steps"].(float64) != 0 {
		t.Fatalf("retransmit changed state: %v", out2)
	}
	q2 := tipResult(t, out2)["final_quaternion"].([]any)
	for i := range q1 {
		if q1[i] != q2[i] {
			t.Fatal("retransmit changed attitude")
		}
	}

	// Same seq, different content: 409 packet_conflict.
	tampered := []map[string]any{{"t": 0.0, "w": []float64{1, 1, 1}}}
	code, out3 := mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(1, tampered, nil))
	if code != http.StatusConflict || out3["cause"] != "packet_conflict" {
		t.Fatalf("conflict: %d %v", code, out3)
	}

	// Same timestamp, different w: 409 duplicate_timestamp.
	bad := []map[string]any{
		{"t": 0.0, "w": []float64{0, 0, 0}},
		{"t": 0.5, "w": []float64{0, 0, 0}},
	}
	code, out4 := mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(99, bad, nil))
	if code != http.StatusConflict || out4["cause"] != "duplicate_timestamp" {
		t.Fatalf("timestamp conflict: %d %v", code, out4)
	}
}

// TestTrajectoryHTTPLateInsertAndRecompute verifies an out-of-order packet is
// accepted, reports a partial recompute, and the tip equals one-shot over the
// merged series.
func TestTrajectoryHTTPLateInsertAndRecompute(t *testing.T) {
	r := newRouter()
	id := openTraj(t, r, nil)
	// Samples 0..39, then 50..99 (hole 40..49), then late 40..49.
	mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(1, yawSamples(0.3, 0, 40, 0.05), nil))
	mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(2, yawSamples(0.3, 50, 100, 0.05), nil))
	code, out := mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(3, yawSamples(0.3, 40, 50, 0.05), nil))
	if code != http.StatusOK {
		t.Fatalf("late: %d %v", code, out)
	}
	if rs := out["recomputed_steps"].(float64); rs != 60 {
		t.Fatalf("recomputed_steps = %v, want 60 (not 99)", rs)
	}

	// One-shot reference over the merged 100 samples.
	_, one := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate",
		yawSeriesPayload(0.3, 100, 0.05))
	gq := tipResult(t, out)["final_quaternion"].([]any)
	oq := one["final_quaternion"].([]any)
	for i := range oq {
		if gq[i].(float64) != oq[i].(float64) {
			t.Fatalf("late tip %v != one-shot %v", gq, oq)
		}
	}
}

// TestTrajectoryHTTPLateTooOld verifies the 200-sample window and untouched
// state.
func TestTrajectoryHTTPLateTooOld(t *testing.T) {
	r := newRouter()
	id := openTraj(t, r, nil)
	// 250 samples, first timestamp (i=0) will be the late hole target.
	samples := make([]map[string]any, 0, 249)
	for i := 1; i < 250; i++ {
		samples = append(samples, map[string]any{"t": float64(i) * 0.01, "w": []float64{0, 0, 0.2}})
	}
	mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id), packetBody(1, samples, nil))
	_, before := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s", id), nil)

	late := []map[string]any{{"t": 0.0, "w": []float64{0, 0, 0.2}}}
	code, out := mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id), packetBody(2, late, nil))
	if code != http.StatusUnprocessableEntity || out["cause"] != "late_packet_out_of_window" {
		t.Fatalf("window: %d %v", code, out)
	}
	_, after := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s", id), nil)
	if after["version"] != before["version"] || after["sample_count"] != before["sample_count"] {
		t.Fatal("state changed after window rejection")
	}
}

// TestTrajectoryHTTPInvalidPacketRollback verifies whole-packet atomicity at
// the HTTP boundary.
func TestTrajectoryHTTPInvalidPacketRollback(t *testing.T) {
	r := newRouter()
	id := openTraj(t, r, nil)
	mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(1, yawSamples(0.5, 0, 20, 0.1), nil))
	_, before := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s", id), nil)

	cases := []struct {
		name   string
		body   map[string]any
		raw    string // raw JSON body, used when map encoding cannot represent it
		cause  string
		status int
	}{
		{"missing seq", map[string]any{"samples": yawSamples(0.5, 20, 21, 0.1)}, "", "invalid_packet", http.StatusBadRequest},
		{"non positive step", packetBody(2, []map[string]any{
			{"t": 2.0, "w": []float64{0, 0, 0.5}},
			{"t": 2.0, "w": []float64{0, 0, 0.5}},
		}, nil), "", "non_positive_step", http.StatusBadRequest},
		// A bare NaN token is not legal JSON; binding rejects it with 400 before
		// any state change. The invalid_rate validation path itself is covered
		// at the trajectory package level (TestAtomicRejection "nan rate").
		{"nan token", nil, `{"seq":3,"samples":[{"t":2.0,"w":[NaN,0,0]}]}`, "bad_json", http.StatusBadRequest},
		{"length mismatch", map[string]any{
			"seq":                5,
			"timestamps":         []float64{2.0, 2.1},
			"angular_velocities": [][]float64{{0, 0, 0.5}},
		}, "", "length_mismatch", http.StatusBadRequest},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			var code int
			var out map[string]any
			if tc.raw != "" {
				req := httptest.NewRequest(http.MethodPost,
					fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
					strings.NewReader(tc.raw))
				req.Header.Set("Content-Type", "application/json")
				rec := httptest.NewRecorder()
				r.ServeHTTP(rec, req)
				code = rec.Code
				if err := json.Unmarshal(rec.Body.Bytes(), &out); err != nil {
					t.Fatalf("bad response json: %v (%s)", err, rec.Body.String())
				}
			} else {
				code, out = mustJSON(t, r, http.MethodPost,
					fmt.Sprintf("/api/v1/trajectories/%s/packets", id), tc.body)
			}
			if code != tc.status || out["cause"] != tc.cause {
				t.Fatalf("%s: %d %v", tc.name, code, out)
			}
		})
	}

	// An empty packet carries no new samples: it is a content-free no-op
	// (duplicate-style), applies nothing and leaves state untouched.
	code, empty := mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(40, []map[string]any{}, nil))
	if code != http.StatusOK || !empty["duplicate"].(bool) {
		t.Fatalf("empty packet: %d %v", code, empty)
	}
	_, after := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s", id), nil)
	if after["version"] != before["version"] || after["sample_count"] != before["sample_count"] {
		t.Fatal("invalid packets left partial state")
	}
}

// TestTrajectoryHTTPStrictRollback: strict mode over-threshold packet is 422
// and rolls back.
func TestTrajectoryHTTPStrictRollback(t *testing.T) {
	r := newRouter()
	id := openTraj(t, r, map[string]any{
		"q0":                  []float64{1, 0, 0, 0},
		"max_step_norm_drift": 1e-18,
		"strict_drift":        true,
	})
	fine := []map[string]any{
		{"t": 0, "w": []float64{0, 0, 0.5}},
		{"t": 0.0001, "w": []float64{0, 0, 0.5}},
	}
	mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id), packetBody(1, fine, nil))
	_, before := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s", id), nil)

	bad := []map[string]any{{"t": 1.0, "w": []float64{0, 0, 0.5}}}
	code, out := mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id), packetBody(2, bad, nil))
	if code != http.StatusUnprocessableEntity || out["cause"] != "norm_drift_exceeded" {
		t.Fatalf("strict: %d %v", code, out)
	}
	_, after := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s", id), nil)
	if after["version"] != before["version"] || after["sample_count"] != before["sample_count"] {
		t.Fatal("strict breach did not roll back")
	}
}

// TestTrajectoryHTTPHistoryQuery checks exact and interpolated queries and
// out-of-range rejection, comparing the interpolated result to the truncated
// one-shot endpoint.
func TestTrajectoryHTTPHistoryQuery(t *testing.T) {
	r := newRouter()
	id := openTraj(t, r, nil)
	// 51 samples t=0..5s.
	mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(1, yawSamples(0.5, 0, 51, 0.1), nil))

	// Exact sample time.
	code, out := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s/attitude?t=2.0", id), nil)
	if code != http.StatusOK || out["interpolated"].(bool) {
		t.Fatalf("exact query: %d %v", code, out)
	}
	if out["global_step"].(float64) != 20 {
		t.Fatalf("global step = %v", out["global_step"])
	}

	// Interior t=2.03: compare to one-shot truncated to [0..2.0] plus an
	// interpolated sample at 2.03 (rate constant => interpolation trivial but
	// the code path is the generic one).
	tm := 2.03
	code, qout := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s/attitude?t=%v", id, tm), nil)
	if code != http.StatusOK || !qout["interpolated"].(bool) {
		t.Fatalf("interior query: %d %v", code, qout)
	}
	truncSamples := make([]map[string]any, 22)
	for i := 0; i <= 20; i++ {
		truncSamples[i] = map[string]any{"t": float64(i) * 0.1, "w": []float64{0, 0, 0.5}}
	}
	truncSamples[21] = map[string]any{"t": tm, "w": []float64{0, 0, 0.5}}
	_, ref := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate",
		map[string]any{"q0": []float64{1, 0, 0, 0}, "samples": truncSamples})
	gq := qout["quaternion"].([]any)
	rq := ref["final_quaternion"].([]any)
	for i := range rq {
		if gq[i].(float64) != rq[i].(float64) {
			t.Fatalf("interior quaternion %v != truncated one-shot %v", gq, rq)
		}
	}

	// Out of range -> 422 with cause; malformed t -> 400.
	for _, bad := range []string{"-0.1", "5.01"} {
		rc, oor := mustJSON(t, r, http.MethodGet,
			fmt.Sprintf("/api/v1/trajectories/%s/attitude?t=%s", id, bad), nil)
		if rc != http.StatusUnprocessableEntity || oor["cause"] != "query_out_of_range" {
			t.Fatalf("t=%s: code=%d cause=%v", bad, rc, oor["cause"])
		}
	}
	badCode, badOut := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s/attitude?t=abc", id), nil)
	if badCode != http.StatusBadRequest {
		t.Fatalf("malformed t: %d %v", badCode, badOut)
	}
}

// TestTrajectoryHTTPVersionConflict exercises optimistic concurrency.
func TestTrajectoryHTTPVersionConflict(t *testing.T) {
	r := newRouter()
	id := openTraj(t, r, nil)
	_, out := mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(1, yawSamples(0.5, 0, 20, 0.1), nil))
	v := int64(out["version"].(float64))

	stale := v - 1
	code, cerr := mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(2, yawSamples(0.5, 20, 30, 0.1), &stale))
	if code != http.StatusConflict || cerr["cause"] != "version_conflict" {
		t.Fatalf("version conflict: %d %v", code, cerr)
	}
	// Correct version applies.
	code, _ = mustJSON(t, r, http.MethodPost,
		fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
		packetBody(2, yawSamples(0.5, 20, 30, 0.1), &v))
	if code != http.StatusOK {
		t.Fatalf("matched version: %d", code)
	}
}

// TestTrajectoryHTTPConcurrentAppends serializes same-trajectory appends:
// final sample count and tip match the serial one-shot result, no seq applied
// twice (re-seen seqs return duplicate, not error — here seqs are distinct).
func TestTrajectoryHTTPConcurrentAppends(t *testing.T) {
	r := newRouter()
	id := openTraj(t, r, nil)
	const npkt = 20
	const n = 200
	dt := 10.0 / float64(n-1)
	size := n / npkt

	var wg sync.WaitGroup
	errs := make(chan error, npkt)
	for i := 0; i < npkt; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			lo, hi := i*size, (i+1)*size
			if i == npkt-1 {
				hi = n
			}
			samples := make([]map[string]any, hi-lo)
			for k := lo; k < hi; k++ {
				samples[k-lo] = map[string]any{"t": float64(k) * dt, "w": []float64{0, 0, 0.4}}
			}
			code, out, err := doJSON(r, http.MethodPost,
				fmt.Sprintf("/api/v1/trajectories/%s/packets", id),
				packetBody(int64(i+1), samples, nil))
			if err != nil || code != http.StatusOK {
				errs <- fmt.Errorf("pkt %d: code=%d err=%v out=%v", i, code, err, out)
			}
		}(i)
	}
	wg.Wait()
	close(errs)
	for e := range errs {
		t.Fatal(e)
	}

	_, st := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s", id), nil)
	if st["sample_count"].(float64) != n {
		t.Fatalf("sample count = %v, want %d", st["sample_count"], n)
	}
	if st["applied_packet_count"].(float64) != npkt {
		t.Fatalf("applied = %v, want %d", st["applied_packet_count"], npkt)
	}

	// Tip matches one-shot.
	samples := make([]map[string]any, n)
	for k := 0; k < n; k++ {
		samples[k] = map[string]any{"t": float64(k) * dt, "w": []float64{0, 0, 0.4}}
	}
	_, one := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate",
		map[string]any{"q0": []float64{1, 0, 0, 0}, "samples": samples})
	gq := st["final_quaternion"].([]any)
	oq := one["final_quaternion"].([]any)
	for i := range oq {
		if gq[i].(float64) != oq[i].(float64) {
			t.Fatalf("concurrent tip %v != one-shot %v", gq, oq)
		}
	}
}

// TestTrajectoryHTTPLifecycle covers list, close=delete, and not-found.
func TestTrajectoryHTTPLifecycle(t *testing.T) {
	r := newRouter()
	id := openTraj(t, r, nil)

	code, out := mustJSON(t, r, http.MethodGet, "/api/v1/trajectories", nil)
	if code != http.StatusOK || len(out["trajectories"].([]any)) != 1 {
		t.Fatalf("list: %d %v", code, out)
	}

	// Unknown id.
	code, out = mustJSON(t, r, http.MethodGet,
		"/api/v1/trajectories/deadbeefdeadbeefdeadbeefdeadbeef", nil)
	if code != http.StatusNotFound {
		t.Fatalf("get missing: %d %v", code, out)
	}

	code, _ = mustJSON(t, r, http.MethodDelete,
		fmt.Sprintf("/api/v1/trajectories/%s", id), nil)
	if code != http.StatusOK {
		t.Fatalf("delete: %d", code)
	}
	code, out = mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s", id), nil)
	if code != http.StatusNotFound || out["cause"] != "trajectory_not_found" {
		t.Fatalf("after delete: %d %v", code, out)
	}
	code, _ = mustJSON(t, r, http.MethodDelete,
		fmt.Sprintf("/api/v1/trajectories/%s", id), nil)
	if code != http.StatusNotFound {
		t.Fatalf("double delete: %d", code)
	}
}

// TestTrajectoryHTTPBadCreate checks creation-time validation reuses the same
// rules as one-shot.
func TestTrajectoryHTTPBadCreate(t *testing.T) {
	r := newRouter()
	code, out := mustJSON(t, r, http.MethodPost, "/api/v1/trajectories",
		map[string]any{"q0": []float64{1, 1, 0, 0}})
	if code != http.StatusBadRequest || out["cause"] != "invalid_quaternion" {
		t.Fatalf("bad q0: %d %v", code, out)
	}
	code, out = mustJSON(t, r, http.MethodPost, "/api/v1/trajectories",
		map[string]any{"q0": []float64{1, 0, 0, 0}, "max_step_norm_drift": -1})
	if code != http.StatusBadRequest || out["cause"] != "invalid_threshold" {
		t.Fatalf("bad threshold: %d %v", code, out)
	}
}
