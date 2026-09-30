package trajectory_test

import (
	"fmt"
	"math"
	"math/rand"
	"sync"
	"testing"

	"github.com/example/attitude-service/internal/integrator"
	"github.com/example/attitude-service/internal/quaternion"
	"github.com/example/attitude-service/internal/trajectory"
)

func q0() quaternion.Q { return quaternion.Identity() }

// buildSeries creates a deterministic, non-trivial 3-axis time series with
// piecewise-linear angular velocity so the interpolated midpoints are not
// trivially constant.
func buildSeries(n int, dt float64) []integrator.Sample {
	s := make([]integrator.Sample, n)
	for i := 0; i < n; i++ {
		t := float64(i) * dt
		s[i] = integrator.Sample{
			T: t,
			W: [3]float64{
				0.20 * math.Sin(0.7*t),
				0.15 * math.Cos(0.5*t),
				0.30 + 0.05*math.Sin(1.3*t),
			},
		}
	}
	return s
}

func sameQ(t *testing.T, tag string, got, want quaternion.Q) {
	t.Helper()
	if got != want {
		t.Fatalf("%s quaternion mismatch:\n got %v\nwant %v", tag, got, want)
	}
}

func oneShot(samples []integrator.Sample, strict bool, threshold float64) *integrator.Result {
	if threshold == 0 {
		threshold = integrator.DefaultMaxStepNormDrift
	}
	res, err := integrator.Simulate(q0(), samples, integrator.Options{
		MaxStepNormDrift: threshold,
		StrictDrift:      strict,
	})
	if err != nil {
		panic(fmt.Sprintf("reference Simulate failed: %v", err))
	}
	return res
}

// splitAt cuts samples into packets at the given sorted boundary indices
// (indices are end-of-packet cut points over 1..n-1).
func splitAt(samples []integrator.Sample, cuts []int) [][]integrator.Sample {
	packets := make([][]integrator.Sample, 0, len(cuts)+1)
	start := 0
	for _, c := range cuts {
		packets = append(packets, samples[start:c])
		start = c
	}
	packets = append(packets, samples[start:])
	return packets
}

func openTrajectory(strict bool, threshold float64) *trajectory.Trajectory {
	reg := trajectory.NewRegistry()
	return reg.Create(trajectory.Config{
		Initial:          q0(),
		MaxStepNormDrift: threshold,
		StrictDrift:      strict,
	})
}

func appendPackets(t *testing.T, tr *trajectory.Trajectory, packets [][]integrator.Sample, seq0 int64) *trajectory.AppendResult {
	t.Helper()
	var res *trajectory.AppendResult
	for i, p := range packets {
		var err *trajectory.Error
		res, err = tr.Append(seq0+int64(i), nil, p)
		if err != nil {
			t.Fatalf("append packet %d failed: %v", seq0+int64(i), err)
		}
	}
	return res
}

// ---- 验收一：切包无关，逐位相同 --------------------------------------------

func TestRandomPacketSplittingBitIdentical(t *testing.T) {
	const trials = 200
	rng := rand.New(rand.NewSource(20260930))
	samples := buildSeries(120, 0.07)
	ref := oneShot(samples, false, 0)

	for trial := 0; trial < trials; trial++ {
		tr := openTrajectory(false, 0)

		// Random ordered cuts, including empty cuts (single-sample packets).
		var cuts []int
		for i := 1; i < len(samples); i++ {
			if rng.Intn(2) == 0 {
				cuts = append(cuts, i)
			}
		}
		packets := splitAt(samples, cuts)

		// Sometimes force a minimum packet of exactly one sample.
		if trial%5 == 0 && len(cuts) > 1 {
			packets = splitAt(samples, []int{cuts[0], cuts[0] + 1})
		}

		res := appendPackets(t, tr, packets, 0)
		sameQ(t, fmt.Sprintf("trial %d (packets=%d)", trial, len(packets)), res.Final, ref.Final)
		if res.MaxDrift != ref.MaxDrift {
			t.Fatalf("trial %d: max drift %v != %v", trial, res.MaxDrift, ref.MaxDrift)
		}
		if res.SampleCount != len(samples) {
			t.Fatalf("trial %d: sample count %d", trial, res.SampleCount)
		}
		if res.StepCount != len(samples)-1 {
			t.Fatalf("trial %d: step count %d", trial, res.StepCount)
		}
	}
}

// TestCrossPacketInterpolationMatches verifies explicitly that the step
// straddling a packet boundary uses the exact same interpolation as the
// adjacent samples in a one-shot run (it must, as both call integrator.Step).
func TestCrossPacketInterpolationMatches(t *testing.T) {
	samples := buildSeries(10, 0.13)
	ref := oneShot(samples, false, 0)

	tr := openTrajectory(false, 0)
	// Boundary lands in the middle of the 3->4 step.
	res := appendPackets(t, tr, splitAt(samples, []int{4}), 0)
	sameQ(t, "cross-boundary", res.Final, ref.Final)
}

// ---- 验收二：晚到的包认回去，只重算尾部 -------------------------------------

func TestLatePacketMergesAndRecomputesTail(t *testing.T) {
	samples := buildSeries(300, 0.05) // ordered arrival would be 0..299

	tr := openTrajectory(false, 0)
	// Send everything except samples 270..274 (a late retransmission).
	head := samples[:270]
	rest := samples[275:]
	appendPackets(t, tr, [][]integrator.Sample{head, rest}, 1)

	before := tr.Info()
	late := samples[270:275]
	res, err := tr.Append(900, nil, late)
	if err != nil {
		t.Fatalf("late packet rejected: %v", err)
	}

	ref := oneShot(samples, false, 0)
	sameQ(t, "after late merge", res.Final, ref.Final)
	if res.MaxDrift != ref.MaxDrift {
		t.Fatalf("max drift after late merge %v != %v", res.MaxDrift, ref.MaxDrift)
	}
	// 5 inserted + 25 pre-existing samples after insertion point => 30 steps
	// (global steps 270..299), never the full 299 steps from scratch.
	if res.RecomputedSteps != 30 {
		t.Fatalf("recomputed steps = %d, want 30", res.RecomputedSteps)
	}
	if res.RecomputedSteps >= len(samples)-1 {
		t.Fatal("late packet triggered a from-scratch recompute")
	}
	if before.SampleCount+5 != res.SampleCount {
		t.Fatalf("sample count after late merge = %d, want %d", res.SampleCount, before.SampleCount+5)
	}
}

func TestLatePacketBeyond200SamplesRejected(t *testing.T) {
	samples := buildSeries(500, 0.05)
	tr := openTrajectory(false, 0)
	appendPackets(t, tr, [][]integrator.Sample{samples}, 1)

	before := tr.Info()
	late := []integrator.Sample{{T: samples[100].T + 0.001, W: [3]float64{0.1, 0, 0}}}
	_, err := tr.Append(1234, nil, late)
	if err == nil {
		t.Fatal("out-of-window late packet was accepted")
	}
	if err.Cause != trajectory.CauseLateOutOfWindow {
		t.Fatalf("cause = %s, want %s", err.Cause, trajectory.CauseLateOutOfWindow)
	}
	after := tr.Info()
	if after.SampleCount != before.SampleCount || after.Version != before.Version ||
		after.MaxDrift != before.MaxDrift {
		t.Fatal("rejected late packet changed trajectory state")
	}
}

// TestLatePacketWindowBoundary pins the ±200-sample retransmission window.
func TestLatePacketWindowBoundary(t *testing.T) {
	// Existing series length 401: head 0..201, tail 202..400. Inserting
	// sample 202 leaves exactly 199 existing samples behind it — the insertion
	// lands on the 200th most recent sample and must be accepted.
	samples := buildSeries(402, 0.05)
	tr := openTrajectory(false, 0)
	appendPackets(t, tr, [][]integrator.Sample{samples[:202], samples[203:]}, 1)
	res, err := tr.Append(77, nil, []integrator.Sample{samples[202]})
	if err != nil {
		t.Fatalf("within-window late packet rejected: %v", err)
	}
	// Global steps 202..401 = 200 steps recomputed, well under the full run.
	if res.RecomputedSteps != 200 {
		t.Fatalf("recomputed = %d, want 200", res.RecomputedSteps)
	}

	// Same series length but insert one sample further back: now 200 existing
	// samples lie behind the insertion, i.e. the packet would have to be among
	// the most recent 201 — rejected, trajectory untouched.
	samples2 := buildSeries(403, 0.05)
	tr2 := openTrajectory(false, 0)
	before := appendPackets(t, tr2, [][]integrator.Sample{samples2[:202], samples2[203:]}, 1).SampleCount
	_, err = tr2.Append(78, nil, []integrator.Sample{samples2[202]})
	if err == nil || err.Cause != trajectory.CauseLateOutOfWindow {
		t.Fatalf("200 behind: want out-of-window, got %v", err)
	}
	if tr2.Info().SampleCount != before {
		t.Fatal("rejected packet changed sample count")
	}
}

// TestLatePacketWarningsUseGlobalStepNumbers verifies acceptance item one's
// warning-numbering clause: after a late insertion the non-strict warnings
// must be the same messages, with the same GLOBAL step numbers, as a one-shot
// integration of the merged series.
func TestLatePacketWarningsUseGlobalStepNumbers(t *testing.T) {
	threshold := 1e-12 // coarse steps below will breach it
	full := []integrator.Sample{
		{T: 0, W: [3]float64{0.5, 0.6, 0.7}},
		{T: 2, W: [3]float64{0.5, 0.6, 0.7}},
		{T: 4, W: [3]float64{0.5, 0.6, 0.7}},
		{T: 6, W: [3]float64{0.9, 0.1, 0.2}}, // withheld, sent late
		{T: 8, W: [3]float64{0.9, 0.1, 0.2}},
		{T: 10, W: [3]float64{0.9, 0.1, 0.2}},
	}
	ref := oneShot(full, false, threshold)

	tr := openTrajectory(false, threshold)
	if _, err := tr.Append(1, nil, full[:3]); err != nil {
		t.Fatal(err)
	}
	if _, err := tr.Append(2, nil, full[4:]); err != nil {
		t.Fatal(err)
	}
	res, err := tr.Append(3, nil, []integrator.Sample{full[3]})
	if err != nil {
		t.Fatal(err)
	}
	if len(res.Warnings) != len(ref.Warnings) {
		t.Fatalf("warning count %d != %d", len(res.Warnings), len(ref.Warnings))
	}
	for i, msg := range ref.Warnings {
		if res.Warnings[i].Message != msg {
			t.Fatalf("warning %d:\n got %q\nwant %q", i, res.Warnings[i].Message, msg)
		}
		if res.Warnings[i].Step != i+1 {
			t.Fatalf("warning %d step = %d, want global %d", i, res.Warnings[i].Step, i+1)
		}
	}
	sameQ(t, "late-with-warnings", res.Final, ref.Final)
}

// ---- 验收三：重传幂等与冲突 -------------------------------------------------

func TestDuplicatePacketIdempotent(t *testing.T) {
	samples := buildSeries(40, 0.1)
	tr := openTrajectory(false, 0)
	first, _ := tr.Append(7, nil, samples)
	again, err := tr.Append(7, nil, samples)
	if err != nil {
		t.Fatalf("retransmission rejected: %v", err)
	}
	if !again.Duplicate {
		t.Fatal("identical retransmission not flagged duplicate")
	}
	if again.Version != first.Version {
		t.Fatal("duplicate packet bumped the version")
	}
	sameQ(t, "duplicate", again.Final, first.Final)
	if again.SampleCount != first.SampleCount {
		t.Fatal("duplicate packet changed sample count")
	}
}

func TestSameSeqDifferentContentConflict(t *testing.T) {
	samples := buildSeries(40, 0.1)
	tr := openTrajectory(false, 0)
	if _, err := tr.Append(7, nil, samples); err != nil {
		t.Fatal(err)
	}
	tampered := make([]integrator.Sample, len(samples))
	copy(tampered, samples)
	tampered[10].W = [3]float64{9, 9, 9}

	before := tr.Info()
	_, err := tr.Append(7, nil, tampered)
	if err == nil || err.Cause != trajectory.CausePacketConflict {
		t.Fatalf("want packet_conflict, got %v", err)
	}
	after := tr.Info()
	if after.Version != before.Version || after.SampleCount != before.SampleCount {
		t.Fatal("conflicting packet changed state")
	}
}

func TestTimestampRateConflictRejected(t *testing.T) {
	samples := buildSeries(40, 0.1)
	tr := openTrajectory(false, 0)
	appendPackets(t, tr, [][]integrator.Sample{samples}, 1)

	// Late packet at an existing timestamp with different angular velocity.
	bad := []integrator.Sample{
		{T: samples[15].T, W: [3]float64{1, 2, 3}},
		{T: samples[15].T + 0.01, W: [3]float64{0, 0, 0.1}},
	}
	_, err := tr.Append(55, nil, bad)
	if err == nil || err.Cause != trajectory.CauseTimestampConflict {
		t.Fatalf("want timestamp_conflict, got %v", err)
	}
}

// TestDuplicateContentUnderNewSeqIsIdempotent: a packet whose samples are all
// already present (matching data) under a fresh seq number adds nothing.
func TestDuplicateContentUnderNewSeqIsIdempotent(t *testing.T) {
	samples := buildSeries(20, 0.1)
	tr := openTrajectory(false, 0)
	first, _ := tr.Append(1, nil, samples[:15])
	res, err := tr.Append(2, nil, samples[5:15]) // all timestamps already known, same data
	if err != nil {
		t.Fatal(err)
	}
	if !res.Duplicate {
		t.Fatal("fully redundant packet should be flagged duplicate")
	}
	if res.SampleCount != first.SampleCount || res.Version != first.Version {
		t.Fatal("redundant packet changed state")
	}
}

// ---- 验收四：整包原子（严格模式超阈回滚、非法采样在 validation 拦截） -------

func TestStrictDriftPacketRollsBack(t *testing.T) {
	// Large dt + tight threshold forces a drift breach inside the packet.
	samples := buildSeries(4, 5.0)
	tr := openTrajectory(true, 1e-18)

	_, err := tr.Append(1, nil, samples)
	if err == nil || err.Cause != trajectory.CauseNormDriftExceeded {
		t.Fatalf("want norm_drift_exceeded, got %v", err)
	}
	info := tr.Info()
	if info.SampleCount != 0 || info.MaxDrift != 0 || info.Version != 0 {
		t.Fatalf("strict breach left partial state: %+v", info)
	}
	if len(info.Warnings) != 0 {
		t.Fatal("strict breach left warnings")
	}
}

// TestStrictDriftOnLatePacketRollsBack ensures that a late packet that only
// breaches in a RECOMPUTED tail step also rolls back completely.
func TestStrictDriftOnLatePacketRollsBack(t *testing.T) {
	tr := openTrajectory(true, integrator.DefaultMaxStepNormDrift)
	ok := []integrator.Sample{
		{T: 0, W: [3]float64{0, 0, 0.01}},
		{T: 0.01, W: [3]float64{0, 0, 0.01}},
		{T: 1.0, W: [3]float64{0, 0, 0.01}},
		{T: 2.0, W: [3]float64{0, 0, 0.01}},
	}
	if _, err := tr.Append(1, nil, ok); err != nil {
		t.Fatal(err)
	}
	before := tr.Info()
	// Insert at t=1.5 a huge rate: the step 1.5 -> 2.0 breaches in recompute.
	late := []integrator.Sample{{T: 1.5, W: [3]float64{0, 0, 50}}}
	_, err := tr.Append(2, nil, late)
	if err == nil || err.Cause != trajectory.CauseNormDriftExceeded {
		t.Fatalf("want drift breach on recompute, got %v", err)
	}
	after := tr.Info()
	if after.SampleCount != before.SampleCount || after.Version != before.Version {
		t.Fatal("recomputed-tail breach left partial state")
	}
}

// ---- 验收五：历史时刻可查 ---------------------------------------------------

func TestAttitudeAtMatchesTruncatedOneShot(t *testing.T) {
	samples := buildSeries(80, 0.09)
	tr := openTrajectory(false, 0)
	appendPackets(t, tr, splitAt(samples, []int{17, 43, 60}), 1)

	rng := rand.New(rand.NewSource(4242))
	for k := 0; k < 60; k++ {
		lo := samples[0].T
		hi := samples[len(samples)-1].T
		at := lo + rng.Float64()*(hi-lo)

		p, err := tr.AttitudeAt(at)
		if err != nil {
			t.Fatalf("AttitudeAt(%v): %v", at, err)
		}

		// Reference: truncate to t and append the interpolated sample.
		var trunc []integrator.Sample
		for _, sm := range samples {
			if sm.T <= at {
				trunc = append(trunc, sm)
			}
		}
		i := len(trunc) // first sample after at
		a := trunc[len(trunc)-1]
		b := samples[i]
		trunc = append(trunc, integrator.Sample{
			T: at,
			W: integrator.InterpolatedRate(a, b, at),
		})
		ref := oneShot(trunc, false, 0)
		sameQ(t, fmt.Sprintf("t=%.6f", at), p.Quaternion, ref.Final)
	}

	// Exact sample times give the archived attitude.
	for i := range samples {
		p, err := tr.AttitudeAt(samples[i].T)
		if err != nil {
			t.Fatal(err)
		}
		if !p.ExactSample {
			t.Fatalf("sample time %v not reported exact", samples[i].T)
		}
		ref := oneShot(samples[:i+1], false, 0)
		sameQ(t, fmt.Sprintf("exact %d", i), p.Quaternion, ref.Final)
	}
}

func TestAttitudeAtOutOfRange(t *testing.T) {
	samples := buildSeries(5, 0.1)
	tr := openTrajectory(false, 0)
	appendPackets(t, tr, [][]integrator.Sample{samples}, 1)

	if _, err := tr.AttitudeAt(samples[0].T - 0.01); err == nil || err.Cause != trajectory.CauseTimeOutOfRange {
		t.Fatalf("early t: %v", err)
	}
	if _, err := tr.AttitudeAt(samples[len(samples)-1].T + 0.01); err == nil || err.Cause != trajectory.CauseTimeOutOfRange {
		t.Fatalf("late t: %v", err)
	}

	empty := openTrajectory(false, 0)
	if _, err := empty.AttitudeAt(0.5); err == nil || err.Cause != trajectory.CauseTrajectoryNotOpenYet {
		t.Fatalf("empty trajectory query: %v", err)
	}
}

// ---- 验收六：并发有序 + 版本号 + 轨迹隔离 ------------------------------------

func TestConcurrentAppendsNoLossNoDoubleApply(t *testing.T) {
	samples := buildSeries(60, 0.08)
	tr := openTrajectory(false, 0)

	// Each worker owns one packet; packets are sent concurrently but merged by
	// timestamp, so arrival order must not matter.
	packets := splitAt(samples, []int{6, 13, 20, 31, 44, 52})

	var wg sync.WaitGroup
	errs := make(chan error, len(packets))
	for i, p := range packets {
		wg.Add(1)
		go func(seq int, pkt []integrator.Sample) {
			defer wg.Done()
			if _, err := tr.Append(int64(seq+1), nil, pkt); err != nil {
				errs <- fmt.Errorf("seq %d: %w", seq, err)
			}
		}(i, p)
	}
	wg.Wait()
	close(errs)
	for e := range errs {
		t.Fatal(e)
	}

	info := tr.Info()
	if info.SampleCount != len(samples) {
		t.Fatalf("samples lost/duplicated: %d vs %d", info.SampleCount, len(samples))
	}
	if info.Version != int64(len(packets)) {
		t.Fatalf("version = %d, want %d", info.Version, len(packets))
	}
	ref := oneShot(samples, false, 0)
	final, err := tr.AttitudeAt(samples[len(samples)-1].T)
	if err != nil {
		t.Fatal(err)
	}
	sameQ(t, "concurrent final", final.Quaternion, ref.Final)
}

func TestOptimisticVersionConflict(t *testing.T) {
	samples := buildSeries(30, 0.1)
	tr := openTrajectory(false, 0)

	v0 := int64(0)
	if _, err := tr.Append(1, &v0, samples[:15]); err != nil {
		t.Fatal(err)
	}
	// Caller still believes version is 0.
	_, err := tr.Append(2, &v0, samples[15:])
	if err == nil || err.Cause != trajectory.CauseVersionConflict {
		t.Fatalf("want version_conflict, got %v", err)
	}
	v1 := int64(1)
	if _, err := tr.Append(2, &v1, samples[15:]); err != nil {
		t.Fatalf("correct version must apply: %v", err)
	}
}

func TestTrajectoriesAreIsolated(t *testing.T) {
	reg := trajectory.NewRegistry()
	a := reg.Create(trajectory.Config{Initial: q0(), MaxStepNormDrift: integrator.DefaultMaxStepNormDrift})
	b := reg.Create(trajectory.Config{Initial: q0(), MaxStepNormDrift: integrator.DefaultMaxStepNormDrift})

	sa := buildSeries(20, 0.1)
	sb := buildSeries(20, 0.1)
	for i := range sb {
		sb[i].W = [3]float64{-0.2, 0.3, 0.1}
	}
	ra, _ := a.Append(1, nil, sa)
	rb, _ := b.Append(1, nil, sb)
	if ra.Final == rb.Final {
		t.Fatal("distinct trajectories converged")
	}
	if a.Info().SampleCount != 20 || b.Info().SampleCount != 20 {
		t.Fatal("cross-trajectory contamination")
	}
}

func TestRegistryLifecycle(t *testing.T) {
	reg := trajectory.NewRegistry()
	tr := reg.Create(trajectory.Config{Initial: q0(), MaxStepNormDrift: 1e-4})
	if got, ok := reg.Get(tr.ID()); !ok || got.ID() != tr.ID() {
		t.Fatal("created trajectory not found")
	}
	if len(reg.IDs()) != 1 {
		t.Fatal("registry listing")
	}
	if !reg.Delete(tr.ID()) {
		t.Fatal("delete reported missing")
	}
	if _, ok := reg.Get(tr.ID()); ok {
		t.Fatal("deleted trajectory still resolvable")
	}
	if reg.Delete(tr.ID()) {
		t.Fatal("double delete reported success")
	}
}
