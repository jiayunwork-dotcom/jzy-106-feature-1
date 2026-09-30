package api_test

import (
	"fmt"
	"math"
	"net/http"
	"sync"
	"testing"
)

// ---- helpers -----------------------------------------------------------------

func createTrajectory(t *testing.T, r http.Handler, body map[string]any) string {
	t.Helper()
	code, out := mustJSON(t, r, http.MethodPost, "/api/v1/trajectories", body)
	if code != http.StatusCreated {
		t.Fatalf("create status=%d out=%v", code, out)
	}
	return out["trajectory_id"].(string)
}

func appendPacket(t *testing.T, r http.Handler, id string, body map[string]any) (int, map[string]any) {
	t.Helper()
	return mustJSON(t, r, http.MethodPost, "/api/v1/trajectories/"+id+"/append", body)
}

func packetBody(seq int64, samples []map[string]any) map[string]any {
	return map[string]any{"packet_seq": seq, "samples": samples}
}

func makeSamples3(n int, dt float64, f func(float64) []float64) []map[string]any {
	out := make([]map[string]any, n)
	for i := 0; i < n; i++ {
		tt := float64(i) * dt
		out[i] = map[string]any{"t": tt, "w": f(tt)}
	}
	return out
}

func quatEqual(a, b []any) bool {
	if len(a) != 4 || len(b) != 4 {
		return false
	}
	for i := range a {
		if a[i].(float64) != b[i].(float64) {
			return false
		}
	}
	return true
}

func threeAxis(tm float64) []float64 {
	return []float64{
		0.2 * math.Sin(0.7*tm),
		0.15 * math.Cos(0.5*tm),
		0.3 + 0.05*math.Sin(1.3*tm),
	}
}

// ---- 验收一（HTTP）：随机切包 vs 整段 ----------------------------------------

func TestTrajectoryPacketSplittingMatchesOneShot(t *testing.T) {
	r := newRouter()
	n, dt := 120, 0.07
	samples := makeSamples3(n, dt, threeAxis)

	// Reference: one-shot submission.
	_, ref := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate",
		map[string]any{"q0": []float64{1, 0, 0, 0}, "samples": samples})

	// One cut at every possible boundary, across many requests is slow; pick a
	// pseudo-random deterministic set plus all single-boundary cases.
	id := createTrajectory(t, r, map[string]any{"q0": []float64{1, 0, 0, 0}})

	cuts := map[int]bool{1: true, 2: true, 7: true, 13: true, 37: true, 71: true, 100: true, 119: true}
	start, seq := 0, int64(1)
	var last map[string]any
	for i := 0; i <= n; i++ {
		if i == n || cuts[i] {
			code, out := appendPacket(t, r, id, packetBody(seq, samples[start:i]))
			if code != http.StatusOK {
				t.Fatalf("append [%d:%d] status=%d out=%v", start, i, code, out)
			}
			last = out
			seq++
			start = i
		}
	}
	if !quatEqual(last["final_quaternion"].([]any), ref["final_quaternion"].([]any)) {
		t.Fatalf("packet final %v != one-shot %v", last["final_quaternion"], ref["final_quaternion"])
	}
	if last["max_norm_drift"].(float64) != ref["max_norm_drift"].(float64) {
		t.Fatal("max drift differs")
	}
	if last["sample_count"].(float64) != float64(n) {
		t.Fatal("sample count")
	}
}

// TestTrajectorySingleSamplePackets: minimum packet size is one sample.
func TestTrajectorySingleSamplePackets(t *testing.T) {
	r := newRouter()
	samples := makeSamples3(30, 0.1, threeAxis)
	_, ref := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate",
		map[string]any{"q0": []float64{1, 0, 0, 0}, "samples": samples})

	id := createTrajectory(t, r, map[string]any{"q0": []float64{1, 0, 0, 0}})
	var last map[string]any
	for i, sm := range samples {
		code, out := appendPacket(t, r, id, packetBody(int64(i+1), []map[string]any{sm}))
		if code != http.StatusOK {
			t.Fatalf("single-sample packet %d: %d %v", i, code, out)
		}
		last = out
	}
	if !quatEqual(last["final_quaternion"].([]any), ref["final_quaternion"].([]any)) {
		t.Fatalf("single-sample packets diverged: %v vs %v", last["final_quaternion"], ref["final_quaternion"])
	}
}

// ---- 验收二（HTTP）：乱序补传 -------------------------------------------------

func TestTrajectoryLatePacketHTTP(t *testing.T) {
	r := newRouter()
	n, dt := 300, 0.05
	samples := makeSamples3(n, dt, threeAxis)
	_, ref := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate",
		map[string]any{"q0": []float64{1, 0, 0, 0}, "samples": samples})

	id := createTrajectory(t, r, map[string]any{"q0": []float64{1, 0, 0, 0}})
	appendPacket(t, r, id, packetBody(1, samples[:270]))
	appendPacket(t, r, id, packetBody(2, samples[275:]))

	code, out := appendPacket(t, r, id, packetBody(99, samples[270:275]))
	if code != http.StatusOK {
		t.Fatalf("late packet: %d %v", code, out)
	}
	if !quatEqual(out["final_quaternion"].([]any), ref["final_quaternion"].([]any)) {
		t.Fatalf("late merge diverged: %v vs %v", out["final_quaternion"], ref["final_quaternion"])
	}
	if rs := out["recomputed_steps"].(float64); rs != 30 || rs >= float64(n-1) {
		t.Fatalf("recomputed_steps = %v", rs)
	}
}

func TestTrajectoryLateOutOfWindowHTTP(t *testing.T) {
	r := newRouter()
	n := 500
	samples := makeSamples3(n, 0.05, threeAxis)
	id := createTrajectory(t, r, map[string]any{"q0": []float64{1, 0, 0, 0}})
	appendPacket(t, r, id, packetBody(1, samples))

	code, out := appendPacket(t, r, id, packetBody(5000, []map[string]any{
		{"t": 5.001, "w": []float64{0.1, 0, 0}},
	}))
	if code != http.StatusUnprocessableEntity || out["cause"] != "late_packet_out_of_window" {
		t.Fatalf("status=%d out=%v", code, out)
	}
	// State untouched.
	_, info := mustJSON(t, r, http.MethodGet, "/api/v1/trajectories/"+id, nil)
	tr := info["trajectory"].(map[string]any)
	if tr["sample_count"].(float64) != float64(n) {
		t.Fatal("rejected packet left state behind")
	}
}

// ---- 验收三（HTTP）：幂等与冲突 ----------------------------------------------

func TestTrajectoryDuplicateAndConflictHTTP(t *testing.T) {
	r := newRouter()
	samples := makeSamples3(40, 0.1, threeAxis)
	id := createTrajectory(t, r, map[string]any{"q0": []float64{1, 0, 0, 0}})
	_, first := appendPacket(t, r, id, packetBody(7, samples))

	_, again := appendPacket(t, r, id, packetBody(7, samples))
	if again["duplicate_packet"] != true {
		t.Fatal("identical retransmission not flagged")
	}
	if again["version"] != first["version"] {
		t.Fatal("duplicate bumped version")
	}

	// Same seq, changed content.
	tampered := make([]map[string]any, len(samples))
	for i, sm := range samples {
		wm := sm["w"].([]float64)
		cp := make([]float64, 3)
		copy(cp, wm)
		tampered[i] = map[string]any{"t": sm["t"], "w": cp}
	}
	tampered[5]["w"] = []float64{9, 9, 9}
	code, out := appendPacket(t, r, id, packetBody(7, tampered))
	if code != http.StatusConflict || out["cause"] != "packet_conflict" {
		t.Fatalf("same-seq conflict: %d %v", code, out)
	}

	// Existing timestamp, mismatched rate.
	code, out = appendPacket(t, r, id, packetBody(8, []map[string]any{
		{"t": samples[10]["t"], "w": []float64{1, 2, 3}},
	}))
	if code != http.StatusConflict || out["cause"] != "timestamp_conflict" {
		t.Fatalf("timestamp conflict: %d %v", code, out)
	}
}

// ---- 验收四（HTTP）：原子回滚 -------------------------------------------------

func TestTrajectoryStrictRollbackHTTP(t *testing.T) {
	r := newRouter()
	id := createTrajectory(t, r, map[string]any{
		"q0":                  []float64{1, 0, 0, 0},
		"max_step_norm_drift": 1e-18,
		"strict_drift":        true,
	})
	bad := []map[string]any{
		{"t": 0.0, "w": []float64{0, 0, 0.5}},
		{"t": 5.0, "w": []float64{0, 0, 0.5}},
	}
	code, out := appendPacket(t, r, id, packetBody(1, bad))
	if code != http.StatusUnprocessableEntity || out["cause"] != "norm_drift_exceeded" {
		t.Fatalf("status=%d out=%v", code, out)
	}
	_, info := mustJSON(t, r, http.MethodGet, "/api/v1/trajectories/"+id, nil)
	tr := info["trajectory"].(map[string]any)
	if tr["sample_count"].(float64) != 0 || tr["max_norm_drift"].(float64) != 0 {
		t.Fatalf("strict packet left partial state: %v", tr)
	}
}

// An invalid sample anywhere in a packet rejects the whole packet with 400.
func TestTrajectoryInvalidPacketRejectedHTTP(t *testing.T) {
	r := newRouter()
	id := createTrajectory(t, r, map[string]any{"q0": []float64{1, 0, 0, 0}})
	appendPacket(t, r, id, packetBody(1, makeSamples3(10, 0.1, threeAxis)))

	cases := []map[string]any{
		packetBody(2, []map[string]any{
			{"t": 1.0, "w": []float64{0, 0, 0.1}},
			{"t": 0.5, "w": []float64{0, 0, 0.1}}, // out of order
		}),
		{"packet_seq": 3, "samples": []map[string]any{}},                     // empty
		{"samples": []map[string]any{{"t": 2.0, "w": []float64{0, 0, 0.1}}}}, // no seq
	}
	wants := []string{"non_positive_step", "empty_sequence", "missing_packet_seq"}
	for i, body := range cases {
		code, out := appendPacket(t, r, id, body)
		if code != http.StatusBadRequest || out["cause"] != wants[i] {
			t.Fatalf("case %d: status=%d out=%v want cause %s", i, code, out, wants[i])
		}
	}
	_, info := mustJSON(t, r, http.MethodGet, "/api/v1/trajectories/"+id, nil)
	tr := info["trajectory"].(map[string]any)
	if tr["sample_count"].(float64) != 10 || tr["version"].(float64) != 1 {
		t.Fatalf("invalid packets changed state: %v", tr)
	}
}

// ---- 验收五（HTTP）：历史时刻查询 --------------------------------------------

func TestTrajectoryAttitudeQueryHTTP(t *testing.T) {
	r := newRouter()
	n, dt := 60, 0.1
	samples := makeSamples3(n, dt, threeAxis)
	id := createTrajectory(t, r, map[string]any{"q0": []float64{1, 0, 0, 0}})
	appendPacket(t, r, id, packetBody(1, samples[:20]))
	appendPacket(t, r, id, packetBody(2, samples[20:]))

	at := 1.234
	code, out := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s/attitude?t=%g", id, at), nil)
	if code != http.StatusOK {
		t.Fatalf("query: %d %v", code, out)
	}
	if out["exact_sample"] != false {
		t.Fatal("interior time must not be an exact sample")
	}

	// Reference: truncate at t with the interpolated rate appended.
	u := (at - 1.2) / 0.1
	trunc := append([]map[string]any{}, samples[:13]...) // t = 0..1.2
	wPrev := threeAxis(1.2)
	wNext := threeAxis(1.3)
	interp := []float64{
		(1-u)*wPrev[0] + u*wNext[0],
		(1-u)*wPrev[1] + u*wNext[1],
		(1-u)*wPrev[2] + u*wNext[2],
	}
	trunc = append(trunc, map[string]any{"t": at, "w": interp})
	_, ref := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate",
		map[string]any{"q0": []float64{1, 0, 0, 0}, "samples": trunc})
	if !quatEqual(out["quaternion"].([]any), ref["final_quaternion"].([]any)) {
		t.Fatalf("query %v != truncated one-shot %v", out["quaternion"], ref["final_quaternion"])
	}

	// Out of range.
	code, out = mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s/attitude?t=%g", id, float64(n)*dt), nil)
	if code != http.StatusBadRequest || out["cause"] != "time_out_of_range" {
		t.Fatalf("out-of-range: %d %v", code, out)
	}

	// Exact sample time.
	code, out = mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s/attitude?t=%g", id, 0.5), nil)
	if code != http.StatusOK || out["exact_sample"] != true {
		t.Fatalf("exact sample: %d %v", code, out)
	}
}

// ---- 验收六（HTTP）：并发追加 + 版本号 + 生命周期 -----------------------------

func TestTrajectoryConcurrentAppendsHTTP(t *testing.T) {
	r := newRouter()
	n := 80
	samples := makeSamples3(n, 0.07, threeAxis)
	_, ref := mustJSON(t, r, http.MethodPost, "/api/v1/attitude/integrate",
		map[string]any{"q0": []float64{1, 0, 0, 0}, "samples": samples})

	id := createTrajectory(t, r, map[string]any{"q0": []float64{1, 0, 0, 0}})

	bounds := []int{5, 11, 20, 33, 41, 55, 66, 72}
	packets := [][]map[string]any{}
	start := 0
	for _, b := range bounds {
		packets = append(packets, samples[start:b])
		start = b
	}
	packets = append(packets, samples[start:])

	var wg sync.WaitGroup
	errs := make(chan error, len(packets))
	for i, p := range packets {
		wg.Add(1)
		go func(seq int, pkt []map[string]any) {
			defer wg.Done()
			code, _, err := doJSON(r, http.MethodPost,
				"/api/v1/trajectories/"+id+"/append", packetBody(int64(seq+1), pkt))
			if err != nil {
				errs <- err
				return
			}
			if code != http.StatusOK {
				errs <- fmt.Errorf("seq %d status %d", seq, code)
			}
		}(i, p)
	}
	wg.Wait()
	close(errs)
	for e := range errs {
		t.Error(e)
	}

	_, info := mustJSON(t, r, http.MethodGet, "/api/v1/trajectories/"+id, nil)
	tr := info["trajectory"].(map[string]any)
	if tr["sample_count"].(float64) != float64(n) {
		t.Fatalf("lost/duplicated samples: %v", tr["sample_count"])
	}
	if tr["version"].(float64) != float64(len(packets)) {
		t.Fatalf("version = %v, want %d", tr["version"], len(packets))
	}
	// Terminal attitude equals the one-shot result.
	code, out := mustJSON(t, r, http.MethodGet,
		fmt.Sprintf("/api/v1/trajectories/%s/attitude?t=%g", id, float64(n-1)*0.07), nil)
	if code != http.StatusOK || !quatEqual(out["quaternion"].([]any), ref["final_quaternion"].([]any)) {
		t.Fatalf("concurrent final diverged: %d %v", code, out)
	}
}

func TestTrajectoryExpectedVersionHTTP(t *testing.T) {
	r := newRouter()
	samples := makeSamples3(30, 0.1, threeAxis)
	id := createTrajectory(t, r, map[string]any{"q0": []float64{1, 0, 0, 0}})

	body := packetBody(1, samples[:15])
	body["expected_version"] = 0
	code, out := appendPacket(t, r, id, body)
	if code != http.StatusOK {
		t.Fatalf("version 0 append: %d %v", code, out)
	}
	// Stale expected version.
	stale := packetBody(2, samples[15:])
	stale["expected_version"] = 0
	code, out = appendPacket(t, r, id, stale)
	if code != http.StatusConflict || out["cause"] != "version_conflict" {
		t.Fatalf("stale version: %d %v", code, out)
	}
	// Current version applies.
	fresh := packetBody(2, samples[15:])
	fresh["expected_version"] = 1
	if code, _ := appendPacket(t, r, id, fresh); code != http.StatusOK {
		t.Fatalf("current version rejected: %d", code)
	}
}

func TestTrajectoryLifecycleHTTP(t *testing.T) {
	r := newRouter()
	id := createTrajectory(t, r, map[string]any{"q0": []float64{1, 0, 0, 0}})

	code, out := mustJSON(t, r, http.MethodGet, "/api/v1/trajectories", nil)
	if code != http.StatusOK || len(out["trajectories"].([]any)) != 1 {
		t.Fatalf("list: %d %v", code, out)
	}
	code, _ = mustJSON(t, r, http.MethodGet, "/api/v1/trajectories/"+id, nil)
	if code != http.StatusOK {
		t.Fatal("get")
	}
	// Unknown id.
	code, out = mustJSON(t, r, http.MethodPost, "/api/v1/trajectories/deadbeef/append",
		packetBody(1, makeSamples3(2, 0.1, threeAxis)))
	if code != http.StatusNotFound || out["cause"] != "trajectory_not_found" {
		t.Fatalf("missing trajectory: %d %v", code, out)
	}
	code, _ = mustJSON(t, r, http.MethodDelete, "/api/v1/trajectories/"+id, nil)
	if code != http.StatusOK {
		t.Fatal("delete")
	}
	code, _ = mustJSON(t, r, http.MethodGet, "/api/v1/trajectories/"+id, nil)
	if code != http.StatusNotFound {
		t.Fatalf("deleted trajectory still visible: %d", code)
	}
}

func TestTrajectoryCreateRejectsBadInputHTTP(t *testing.T) {
	r := newRouter()
	code, out := mustJSON(t, r, http.MethodPost, "/api/v1/trajectories",
		map[string]any{"q0": []float64{2, 0, 0, 0}})
	if code != http.StatusBadRequest || out["cause"] != "invalid_quaternion" {
		t.Fatalf("status=%d out=%v", code, out)
	}
}
