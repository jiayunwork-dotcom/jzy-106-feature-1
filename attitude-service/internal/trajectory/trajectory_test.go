package trajectory_test

import (
	"errors"
	"fmt"
	"math"
	"math/rand"
	"strconv"
	"sync"
	"testing"

	"github.com/example/attitude-service/internal/integrator"
	"github.com/example/attitude-service/internal/quaternion"
	"github.com/example/attitude-service/internal/trajectory"
	"github.com/example/attitude-service/internal/validation"
)

func rateFn(t float64) [3]float64 {
	return [3]float64{
		0.7*math.Sin(1.3*t) + 0.2,
		-0.5*math.Cos(0.9*t) + 0.05*t,
		0.4*math.Sin(0.5*t+0.6) - 0.3,
	}
}

func makeSamples(n int, t0, t1 float64) []integrator.Sample {
	s := make([]integrator.Sample, n)
	for i := 0; i < n; i++ {
		u := float64(i) / float64(n-1)
		t := t0 + u*(t1-t0)
		s[i] = integrator.Sample{T: t, W: rateFn(t)}
	}
	return s
}

func oneShot(t *testing.T, q0 quaternion.Q, samples []integrator.Sample, strict bool) *integrator.Result {
	t.Helper()
	res, err := integrator.Simulate(q0, samples, integrator.Options{StrictDrift: strict})
	if err != nil {
		t.Fatalf("one-shot reference failed: %v", err)
	}
	return res
}

func assertSameQuat(t *testing.T, tag string, got, want quaternion.Q) {
	t.Helper()
	if got != want {
		t.Fatalf("%s: quaternion not bit-identical:\n got  = %+v\n want = %+v", tag, got, want)
	}
}

func assertSameTip(t *testing.T, tag string, m *trajectory.Manager, id string, ref *integrator.Result) {
	t.Helper()
	st, ok := m.Get(id)
	if !ok {
		t.Fatalf("%s: trajectory vanished", tag)
	}
	assertSameQuat(t, tag, st.Final, ref.Final)
	if st.SampleCount != len(ref.Steps)+1 {
		t.Fatalf("%s: sample count %d != %d", tag, st.SampleCount, len(ref.Steps)+1)
	}
	if st.StepCount != len(ref.Steps) {
		t.Fatalf("%s: step count %d != %d", tag, st.StepCount, len(ref.Steps))
	}
	if st.MaxNormDrift != ref.MaxDrift {
		t.Fatalf("%s: max drift %v != %v", tag, st.MaxNormDrift, ref.MaxDrift)
	}
	if len(st.Warnings) != len(ref.Warnings) {
		t.Fatalf("%s: %d warnings %v != %d %v", tag, len(st.Warnings), st.Warnings, len(ref.Warnings), ref.Warnings)
	}
	for i := range ref.Warnings {
		if st.Warnings[i] != ref.Warnings[i] {
			t.Fatalf("%s: warning %d mismatch:\n got %q\nwant %q", tag, i, st.Warnings[i], ref.Warnings[i])
		}
	}
}

func open(t *testing.T, m *trajectory.Manager, strict bool) string {
	t.Helper()
	id, _, err := m.Open(trajectory.Config{
		Q0:          quaternion.Identity(),
		StrictDrift: strict,
	})
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	return id
}

func appendOK(t *testing.T, m *trajectory.Manager, id string, seq int64, chunk []integrator.Sample, ver int64) *trajectory.AppendOutcome {
	t.Helper()
	out, err := m.Append(id, trajectory.Packet{Seq: seq, Samples: chunk}, ver)
	if err != nil {
		t.Fatalf("append seq=%d: %v", seq, err)
	}
	return out
}

// randomCuts returns prefix-length boundaries partitioning n samples into
// chunks of sizes in [1, maxChunk].
func randomCuts(rng *rand.Rand, n, maxChunk int) []int {
	cuts := []int{0}
	for cuts[len(cuts)-1] < n {
		size := 1 + rng.Intn(maxChunk)
		next := cuts[len(cuts)-1] + size
		if next >= n {
			cuts = append(cuts, n)
			break
		}
		cuts = append(cuts, next)
	}
	return cuts
}

// TestRandomPacketizationBitIdentical is acceptance 1: the same series split
// into arbitrarily bounded, in-order packets must finish at the bit-identical
// terminal quaternion / max drift / warnings of the one-shot run — and every
// intermediate prefix must match the one-shot run of that prefix too, which
// also pins the cross-boundary interpolation.
func TestRandomPacketizationBitIdentical(t *testing.T) {
	rng := rand.New(rand.NewSource(20260929))
	q0 := quaternion.Identity()

	for iter := 0; iter < 40; iter++ {
		n := 2 + rng.Intn(150)
		samples := makeSamples(n, 0, 3.0+rng.Float64()*4.0)
		ref := oneShot(t, q0, samples, false)

		m := trajectory.NewManager()
		id := open(t, m, false)

		cuts := randomCuts(rng, n, 1+rng.Intn(8))
		for k := 1; k < len(cuts); k++ {
			chunk := samples[cuts[k-1]:cuts[k]]
			out := appendOK(t, m, id, int64(k), chunk, -1)
			if out.Duplicate {
				t.Fatalf("iter %d: fresh packet marked duplicate", iter)
			}
			wantSteps := cuts[k] - cuts[k-1]
			if cuts[k-1] == 0 {
				wantSteps-- // the first packet carries no incoming step
			}
			if out.RecomputedSteps != wantSteps {
				t.Fatalf("iter %d chunk %d: recomputed %d steps, want %d",
					iter, k, out.RecomputedSteps, wantSteps)
			}
			prefRef := oneShot(t, q0, samples[:cuts[k]], false)
			assertSameTip(t, fmt.Sprintf("iter %d prefix %d", iter, cuts[k]), m, id, prefRef)
		}
		assertSameTip(t, "final", m, id, ref)
	}
}

// TestSingleSamplePackets forces a boundary between every adjacent pair,
// including a one-sample first packet (zero steps) followed by one-step
// packets.
func TestSingleSamplePackets(t *testing.T) {
	samples := makeSamples(120, 0, 5.0)
	ref := oneShot(t, quaternion.Identity(), samples, false)
	m := trajectory.NewManager()
	id := open(t, m, false)
	var ver int64
	for i := range samples {
		out := appendOK(t, m, id, int64(i), samples[i:i+1], ver)
		ver = out.Version
		wantSteps := 0
		if i > 0 {
			wantSteps = 1
		}
		if out.RecomputedSteps != wantSteps {
			t.Fatalf("packet %d recomputed %d steps, want %d", i, out.RecomputedSteps, wantSteps)
		}
	}
	assertSameTip(t, "single-sample packets", m, id, ref)
}

// TestLatePacketReinsertsAndRecomputes is acceptance 2: a packet whose
// timestamps precede the current end is merged at its proper place, only the
// suffix from the nearest archive is recomputed (never from the start), and
// the result is bit-identical to submitting the merged series one-shot.
func TestLatePacketReinsertsAndRecomputes(t *testing.T) {
	samples := makeSamples(100, 0, 8.0)
	// First stream leaves a hole at indices 40..49.
	first := append(append([]integrator.Sample{}, samples[:40]...), samples[50:]...)
	late := samples[40:50]

	m := trajectory.NewManager()
	id := open(t, m, false)
	appendOK(t, m, id, 1, first[:40], -1)
	appendOK(t, m, id, 2, first[40:], -1)

	out3, err := m.Append(id, trajectory.Packet{Seq: 3, Samples: late}, -1)
	if err != nil {
		t.Fatalf("late append: %v", err)
	}
	// Anchor is sample 40 (already archived); steps 40..99 = 60 steps replay,
	// strictly fewer than the 99 total steps.
	if out3.RecomputedSteps != 60 {
		t.Fatalf("recomputed steps = %d, want 60", out3.RecomputedSteps)
	}
	if out3.Duplicate {
		t.Fatal("late packet marked duplicate")
	}
	ref := oneShot(t, quaternion.Identity(), samples, false)
	assertSameTip(t, "after late insert", m, id, ref)

	// A second pattern: head, then tail (a forward packet), then the hole.
	s2 := makeSamples(80, 0, 6.0)
	id2 := open(t, m, false)
	packets := [][2]int{{0, 20}, {60, 80}, {20, 40}, {40, 60}}
	seq := int64(1)
	for _, rg := range packets {
		out := appendOK(t, m, id2, seq, s2[rg[0]:rg[1]], -1)
		if rg[0] == 20 || rg[0] == 40 {
			if out.RecomputedSteps >= len(s2)-1 {
				t.Fatalf("late packet recomputed from start: %d", out.RecomputedSteps)
			}
		}
		seq++
	}
	ref2 := oneShot(t, quaternion.Identity(), s2, false)
	assertSameTip(t, "head-tail-hole pattern", m, id2, ref2)
}

// TestLatePacketBeyondWindowRejected checks the 200-sample look-back limit
// and that the trajectory is left completely untouched; plus the exact
// boundary.
func TestLatePacketBeyondWindowRejected(t *testing.T) {
	m := trajectory.NewManager()
	id := open(t, m, false)
	const N = 400
	old := make([]integrator.Sample, 0, N-1)
	for i := 0; i < N; i++ {
		if i == 5 { // hole to be filled by the late packet
			continue
		}
		t := float64(i) * 0.01
		old = append(old, integrator.Sample{T: t, W: rateFn(t)})
	}
	appendOK(t, m, id, 1, old, -1)
	before, _ := m.Get(id)

	pkt := []integrator.Sample{{T: 0.05, W: rateFn(0.05)}}
	_, err := m.Append(id, trajectory.Packet{Seq: 2, Samples: pkt}, -1)
	var te *trajectory.Error
	if !errors.As(err, &te) || te.Cause != validation.CauseLatePacketTooOld {
		t.Fatalf("err = %v, want late_packet_out_of_window", err)
	}
	after, _ := m.Get(id)
	if after.Version != before.Version || after.SampleCount != before.SampleCount ||
		after.Final != before.Final || after.MaxNormDrift != before.MaxNormDrift {
		t.Fatal("trajectory state changed after out-of-window rejection")
	}

	// Exact boundary: samples t=2..t=201 (200 of them) newer than t=1 is OK.
	m2 := trajectory.NewManager()
	id2 := open(t, m2, false)
	base := make([]integrator.Sample, 0, 201)
	for i := 0; i < 202; i++ {
		if i == 1 {
			continue
		}
		base = append(base, integrator.Sample{T: float64(i), W: [3]float64{0, 0, 0.1}})
	}
	appendOK(t, m2, id2, 1, base, -1)
	boundary := []integrator.Sample{{T: 1, W: [3]float64{0, 0, 0.1}}}
	out, err := m2.Append(id2, trajectory.Packet{Seq: 2, Samples: boundary}, -1)
	if err != nil {
		t.Fatalf("200-newer boundary should be accepted: %v", err)
	}
	if out.RecomputedSteps != 201 {
		t.Fatalf("boundary recompute = %d, want 201", out.RecomputedSteps)
	}
	if st2, _ := m2.Get(id2); st2.SampleCount != 202 {
		t.Fatalf("boundary sample count = %d, want 202", st2.SampleCount)
	}
}

// TestRetransmitIdempotentAndConflicts is acceptance 3.
func TestRetransmitIdempotentAndConflicts(t *testing.T) {
	samples := makeSamples(60, 0, 4.0)
	m := trajectory.NewManager()
	id := open(t, m, false)
	appendOK(t, m, id, 7, samples[:30], -1)
	out2 := appendOK(t, m, id, 8, samples[30:], -1)
	tip := out2.Tip
	ref := oneShot(t, quaternion.Identity(), samples, false)
	assertSameQuat(t, "ref", tip.Final, ref.Final)

	// Same seq, identical content: duplicate no-op, version unchanged.
	out3, err := m.Append(id, trajectory.Packet{Seq: 7, Samples: samples[:30]}, -1)
	if err != nil {
		t.Fatalf("retransmit: %v", err)
	}
	if !out3.Duplicate || out3.Version != out2.Version || out3.RecomputedSteps != 0 {
		t.Fatalf("retransmit not idempotent: %+v", out3)
	}
	if out3.Tip.Final != tip.Final {
		t.Fatal("duplicate packet changed tip")
	}

	// Same seq, different content: conflict.
	tampered := []integrator.Sample{{T: samples[0].T, W: [3]float64{9, 9, 9}}}
	_, err = m.Append(id, trajectory.Packet{Seq: 7, Samples: tampered}, -1)
	var ce *trajectory.Error
	if !errors.As(err, &ce) || ce.Cause != validation.CausePacketConflict {
		t.Fatalf("err = %v, want packet_conflict", err)
	}

	// Existing timestamp with a mismatched rate: timestamp conflict.
	bad := []integrator.Sample{
		{T: samples[0].T, W: [3]float64{0, 0, 0}},
		{T: samples[0].T + 0.123, W: [3]float64{0, 0, 0}},
	}
	_, err = m.Append(id, trajectory.Packet{Seq: 99, Samples: bad}, -1)
	if !errors.As(err, &ce) || ce.Cause != validation.CauseDuplicateTimestamp {
		t.Fatalf("err = %v, want duplicate_timestamp", err)
	}
	if wantSub := strconv.FormatFloat(samples[0].T, 'g', -1, 64); !contains(ce.Message, wantSub) {
		t.Fatalf("conflict error must name the timestamp %s: %v", wantSub, ce)
	}

	// Rejected conflicts left no trace.
	st, _ := m.Get(id)
	if st.SampleCount != len(samples) || st.Final != tip.Final {
		t.Fatal("conflict left partial state")
	}
}

func contains(s, sub string) bool {
	return len(sub) > 0 && len(s) >= len(sub) && indexOf(s, sub) >= 0
}

func indexOf(s, sub string) int {
	for i := 0; i+len(sub) <= len(s); i++ {
		if s[i:i+len(sub)] == sub {
			return i
		}
	}
	return -1
}

// TestAtomicRejection is acceptance 4: an invalid packet changes nothing.
func TestAtomicRejection(t *testing.T) {
	samples := makeSamples(60, 0, 4.0)
	m := trajectory.NewManager()
	id := open(t, m, false)
	appendOK(t, m, id, 1, samples[:30], -1)
	stBefore, _ := m.Get(id)

	cases := map[string]struct {
		seq int64
		pkt []integrator.Sample
	}{
		"non positive step": {2, []integrator.Sample{
			{T: samples[29].T + 0.1, W: [3]float64{0, 0, 0.1}},
			{T: samples[29].T + 0.1, W: [3]float64{0, 0, 0.1}},
		}},
		"nan rate": {3, []integrator.Sample{
			{T: samples[29].T + 0.1, W: [3]float64{math.NaN(), 0, 0}},
		}},
		"nan timestamp": {4, []integrator.Sample{
			{T: math.NaN(), W: [3]float64{0, 0, 0}},
		}},
		"packet internal duplicate": {5, []integrator.Sample{
			{T: samples[29].T + 0.2, W: [3]float64{0, 0, 0.1}},
			{T: samples[29].T + 0.2, W: [3]float64{0, 0, 0.1}},
		}},
	}
	for name, tc := range cases {
		_, err := m.Append(id, trajectory.Packet{Seq: tc.seq, Samples: tc.pkt}, -1)
		if err == nil {
			t.Fatalf("%s: expected rejection", name)
		}
		after, _ := m.Get(id)
		if after.Version != stBefore.Version || after.Final != stBefore.Final ||
			after.SampleCount != stBefore.SampleCount || after.MaxNormDrift != stBefore.MaxNormDrift {
			t.Fatalf("%s: rejection was not atomic", name)
		}
	}
}

// TestStrictDriftPacketRollsBack: in strict mode a packet whose steps breach
// the drift threshold is rejected wholesale and the trajectory rolls back.
func TestStrictDriftPacketRollsBack(t *testing.T) {
	m := trajectory.NewManager()
	id, _, err := m.Open(trajectory.Config{
		Q0:               quaternion.Identity(),
		StrictDrift:      true,
		MaxStepNormDrift: 1e-12,
	})
	if err != nil {
		t.Fatal(err)
	}
	ok1 := []integrator.Sample{
		{T: 0, W: [3]float64{0, 0, 0.5}},
		{T: 0.001, W: [3]float64{0, 0, 0.5}},
	}
	appendOK(t, m, id, 1, ok1, -1)
	before, _ := m.Get(id)

	// A coarse step breaches the tight threshold.
	bad := []integrator.Sample{{T: 5.0, W: [3]float64{0, 0, 0.5}}}
	_, gerr := m.Append(id, trajectory.Packet{Seq: 2, Samples: bad}, -1)
	if !errors.Is(gerr, integrator.ErrDriftThreshold) {
		t.Fatalf("err = %v, want drift threshold", gerr)
	}
	after, _ := m.Get(id)
	if after.Version != before.Version || after.Final != before.Final ||
		after.SampleCount != before.SampleCount {
		t.Fatal("strict breach did not roll back")
	}
}

// TestHistoryQuery is acceptance 5: attitudes at sample times and interior
// times match the truncated+resampled one-shot run bit-for-bit.
func TestHistoryQuery(t *testing.T) {
	samples := makeSamples(80, 0.5, 6.5)
	q0 := quaternion.Q{W: 0.5, X: 0.5, Y: 0.5, Z: 0.5}
	m := trajectory.NewManager()
	id, _, err := m.Open(trajectory.Config{Q0: q0})
	if err != nil {
		t.Fatal(err)
	}
	for k, seq := 0, int64(1); k < len(samples); k, seq = k+7, seq+1 {
		end := k + 7
		if end > len(samples) {
			end = len(samples)
		}
		appendOK(t, m, id, seq, samples[k:end], -1)
	}

	// Exact sample times: the archived attitude at that sample.
	for _, idx := range []int{0, 1, 39, 79} {
		pt, qerr := m.AttitudeAt(id, samples[idx].T)
		if qerr != nil {
			t.Fatalf("query exact %d: %v", idx, qerr)
		}
		if pt.Interpolated {
			t.Fatalf("exact sample %d marked interpolated", idx)
		}
		var want quaternion.Q
		if idx == 0 {
			want = q0
		} else {
			want = oneShot(t, q0, samples[:idx+1], false).Final
		}
		assertSameQuat(t, fmt.Sprintf("exact t=%.3f", samples[idx].T), pt.Quaternion, want)
	}

	// Interior times: same linear interpolation the kernel uses, same kernel.
	r := rand.New(rand.NewSource(11))
	for i := 0; i < 30; i++ {
		k := r.Intn(len(samples) - 1)
		u := 0.05 + 0.9*r.Float64()
		tm := samples[k].T + u*(samples[k+1].T-samples[k].T)
		pt, qerr := m.AttitudeAt(id, tm)
		if qerr != nil {
			t.Fatalf("interior query: %v", qerr)
		}
		if !pt.Interpolated {
			t.Fatal("interior query not marked interpolated")
		}
		w := integrator.LinearRateAt(samples[k], samples[k+1], tm)
		trunc := append(append([]integrator.Sample{}, samples[:k+1]...), integrator.Sample{T: tm, W: w})
		want := oneShot(t, q0, trunc, false).Final
		assertSameQuat(t, fmt.Sprintf("interior t=%.6f", tm), pt.Quaternion, want)
	}

	// Out of range on both sides.
	for _, bad := range []float64{0.4999, 6.5001, -1, 100} {
		_, qerr := m.AttitudeAt(id, bad)
		var te *trajectory.Error
		if !errors.As(qerr, &te) || te.Cause != validation.CauseOutOfRangeQuery {
			t.Fatalf("t=%v: err %v, want out_of_range", bad, qerr)
		}
	}
}

// TestOptimisticVersion is part of acceptance 6.
func TestOptimisticVersion(t *testing.T) {
	samples := makeSamples(40, 0, 3.0)
	m := trajectory.NewManager()
	id := open(t, m, false)
	out := appendOK(t, m, id, 1, samples[:20], -1)
	v := out.Version

	_, err := m.Append(id, trajectory.Packet{Seq: 2, Samples: samples[20:25]}, v-1)
	var ve *trajectory.Error
	if !errors.As(err, &ve) || ve.Cause != validation.CauseVersionConflict {
		t.Fatalf("err = %v, want version_conflict", err)
	}
	st, _ := m.Get(id)
	if st.Version != v || st.SampleCount != 20 {
		t.Fatal("stale append was applied")
	}
	appendOK(t, m, id, 2, samples[20:25], v)
}

// TestConcurrentAppends is acceptance 6: concurrent appends to one trajectory
// lose no packet and apply none twice; final state equals the serial one-shot
// over the full series. A second trajectory stays isolated.
func TestConcurrentAppends(t *testing.T) {
	samples := makeSamples(200, 0, 10.0)
	m := trajectory.NewManager()
	id := open(t, m, false)
	idOther := open(t, m, false)

	const npkt = 20
	size := len(samples) / npkt
	var wg sync.WaitGroup
	errs := make(chan error, npkt)
	for i := 0; i < npkt; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			lo := i * size
			hi := lo + size
			if i == npkt-1 {
				hi = len(samples)
			}
			if _, err := m.Append(id, trajectory.Packet{Seq: int64(i + 1), Samples: samples[lo:hi]}, -1); err != nil {
				errs <- fmt.Errorf("pkt %d: %w", i, err)
			}
			if i%5 == 0 {
				if _, err := m.Append(idOther, trajectory.Packet{Seq: int64(i + 1), Samples: samples[lo:hi]}, -1); err != nil {
					errs <- fmt.Errorf("other %d: %w", i, err)
				}
			}
		}(i)
	}
	wg.Wait()
	close(errs)
	for e := range errs {
		t.Fatal(e)
	}

	ref := oneShot(t, quaternion.Identity(), samples, false)
	assertSameTip(t, "concurrent", m, id, ref)
	st, _ := m.Get(id)
	if st.AppliedPacketCount != npkt {
		t.Fatalf("applied packets = %d, want %d", st.AppliedPacketCount, npkt)
	}

	so, _ := m.Get(idOther)
	wantOther := 4 // seqs 1, 6, 11, 16
	if so.AppliedPacketCount != wantOther || so.SampleCount == st.SampleCount {
		t.Fatalf("trajectory isolation broken: other=%+v", so)
	}
}

// TestLifecycle exercises create / view / close.
func TestLifecycle(t *testing.T) {
	m := trajectory.NewManager()
	id := open(t, m, false)
	if _, ok := m.Get(id); !ok {
		t.Fatal("new trajectory not visible")
	}
	if n := len(m.List()); n != 1 {
		t.Fatalf("list = %d trajectories", n)
	}
	if !m.Close(id) {
		t.Fatal("close returned false")
	}
	if m.Close(id) {
		t.Fatal("closing again returned true")
	}
	if _, ok := m.Get(id); ok {
		t.Fatal("closed trajectory still visible")
	}
	if _, err := m.Append(id, trajectory.Packet{Seq: 1, Samples: makeSamples(2, 0, 1)}, -1); err == nil {
		t.Fatal("append after close must fail")
	}
}
