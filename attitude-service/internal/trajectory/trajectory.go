package trajectory

import (
	"math"
	"strconv"
	"sync"

	"github.com/example/attitude-service/internal/integrator"
	"github.com/example/attitude-service/internal/quaternion"
	"github.com/example/attitude-service/internal/validation"
)

// LateWindowSamples bounds how far behind the current end a late
// retransmitted packet may reach: a packet whose earliest sample would be
// inserted when more than LateWindowSamples existing samples are strictly
// newer than it is rejected outright, with the trajectory untouched.
const LateWindowSamples = 200

// Config opens a trajectory. It mirrors the one-shot integration options.
type Config struct {
	Q0               quaternion.Q
	MaxStepNormDrift float64
	StrictDrift      bool
}

// Packet is one telemetry packet: a caller-assigned sequence number plus the
// samples it carries. Samples must be finite with strictly increasing
// timestamps inside the packet; the API layer enforces that with the same
// validators as the one-shot endpoint before the packet reaches this package.
type Packet struct {
	Seq     int64
	Samples []integrator.Sample
}

// Tip is the current trajectory end-point state, field-for-field equivalent
// in meaning to the one-shot integration result's diagnostics.
type Tip struct {
	Final              quaternion.Q
	SampleCount        int
	StepCount          int
	ElapsedTime        float64
	MaxNormDrift       float64
	NormDriftThreshold float64
	Warnings           []string
}

// AppendOutcome reports what one Append did.
type AppendOutcome struct {
	// Duplicate is true when the same packet sequence had already been applied
	// (or accepted) with identical content: nothing was applied.
	Duplicate bool
	// RecomputedSteps is the number of integration steps recomputed from the
	// nearest archive at or before the insertion point. For a duplicate packet
	// it is 0.
	RecomputedSteps int
	Tip             Tip
	// Version is the trajectory version AFTER the append (unchanged on a
	// duplicate no-op).
	Version int64
	// State is a coherent status snapshot taken under the trajectory lock at
	// exactly Version, so an HTTP response never mixes versions when another
	// append races in immediately afterwards.
	State State
}

// State is a defensive snapshot of a trajectory's status.
type State struct {
	ID                 string
	Version            int64
	Initial            quaternion.Q
	MaxStepNormDrift   float64
	StrictDrift        bool
	SampleCount        int
	StepCount          int
	StartTime          float64
	EndTime            float64
	Final              quaternion.Q
	MaxNormDrift       float64
	Warnings           []string
	AppliedPacketCount int
}

// breach is a retained threshold breach of one (global) step.
type breach struct {
	step  int
	drift float64
}

// packetContent binds an already-seen packet sequence to the exact content it
// carried, so a retransmit with different samples is a conflict rather than a
// silent overwrite.
type packetContent struct {
	samples []integrator.Sample
}

type trajectory struct {
	mu        sync.Mutex
	id        string
	q0        quaternion.Q
	threshold float64
	strict    bool
	createdAt int64 // unix seconds, informational only

	// samples and steps align as follows: len(steps) == len(samples)-1;
	// steps[i] is the step samples[i] -> samples[i+1] (global step i+1).
	// samples[i+1] and steps[i].Quaternion are the archive of the post-step
	// state at samples[i+1], so any re-integration can restart there.
	samples []integrator.Sample
	steps   []integrator.StepRecord
	// prefixMax[i] = max norm drift over global steps 1..i+1; len == len(steps).
	prefixMax []float64
	breaches  []breach // retained in step order, later truncated to integrator.MaxWarnings for the Tip

	seen    map[int64]packetContent
	version int64
}

func newTrajectory(id string, cfg Config, now int64) *trajectory {
	threshold := cfg.MaxStepNormDrift
	if threshold <= 0 {
		threshold = integrator.DefaultMaxStepNormDrift
	}
	return &trajectory{
		id:        id,
		q0:        cfg.Q0,
		threshold: threshold,
		strict:    cfg.StrictDrift,
		createdAt: now,
		seen:      make(map[int64]packetContent),
	}
}

// Append merges pkt into the trajectory and re-integrates from the nearest
// archive. All new state is computed into fresh slices BEFORE mutating the
// trajectory, guaranteeing packet atomicity. expectedVersion >= 0 requests an
// optimistic-concurrency check against the current trajectory version.
//
// The caller (Manager) holds the trajectory mutex; this method does no
// locking itself.
func (tr *trajectory) append(pkt Packet, expectedVersion int64) (*AppendOutcome, error) {
	if expectedVersion >= 0 && expectedVersion != tr.version {
		return nil, newError(validation.CauseVersionConflict,
			fmtConflict(expectedVersion, tr.version))
	}

	if prior, ok := tr.seen[pkt.Seq]; ok {
		if samplesEqual(prior.samples, pkt.Samples) {
			return &AppendOutcome{
				Duplicate: true,
				Tip:       tr.tip(),
				Version:   tr.version,
				State:     tr.snapshot(),
			}, nil
		}
		return nil, newError(validation.CausePacketConflict,
			"包序号 "+itoa64(pkt.Seq)+" 已使用但本次内容与首次提交不一致，拒绝整包")
	}

	merged, anchors, inserted, err := tr.merge(pkt)
	if err != nil {
		return nil, err
	}
	if !inserted {
		// Every sample was already present with identical content and the
		// packet sequence itself was new: nothing changes. Record the sequence
		// so a later retransmit is recognized as a duplicate.
		tr.seen[pkt.Seq] = packetContent{samples: copySamples(pkt.Samples)}
		return &AppendOutcome{
			Duplicate: true,
			Tip:       tr.tip(),
			Version:   tr.version,
			State:     tr.snapshot(),
		}, nil
	}

	// Re-integration anchor. merged[:anchors] are samples strictly before the
	// first newly inserted one. The nearest archive at or before that point
	// determines where recomputation starts; the same formula covers every
	// case:
	//
	//   first packet / no completed step yet (anchors == 0, or the trajectory
	//   holds just one sample): start from q0 at merged[0], global steps 1...
	//
	//   forward append (anchors == len(old)): restart from the archived
	//   trajectory end merged[len(old)-1]; only genuinely new steps run, so no
	//   previously integrated step is recomputed.
	//
	//   late insertion (anchors < len(old)): restart from the archive at
	//   merged[anchors-1] (the existing sample immediately before the hole);
	//   the first produced step re-links the bracket to the following sample
	//   and steps 1..anchors-1 stay byte-for-byte untouched.
	oldN := len(tr.samples)
	segStart := anchors // merged index of the segment's anchor sample
	restartQ := tr.q0
	keep := 0 // global steps left untouched before seg.Steps
	switch {
	case anchors == 0 || oldN < 2:
		segStart = 0
	case anchors >= oldN:
		segStart = oldN - 1
		keep = segStart // steps 1..oldN-1 are the existing completed steps
	default:
		segStart = anchors - 1
		keep = segStart // steps 1..anchors-1 unchanged; bridge step is recomputed
	}
	if segStart > 0 {
		restartQ = tr.steps[segStart-1].Quaternion
	}
	segment := merged[segStart:]

	// seg's first produced step is global step keep+1 (Advance numbers the
	// step ending at segment[i] as offset+i).
	seg, err := integrator.Advance(restartQ, segment, integrator.Options{
		MaxStepNormDrift: tr.threshold,
		StrictDrift:      tr.strict,
	}, keep)
	if err != nil {
		// Strict drift rejection: nothing has been mutated yet, so the
		// trajectory is exactly as it was before the packet arrived.
		return nil, err
	}

	// Commit point: computed slices replace old state atomically. Steps
	// 1..keep are unchanged; seg.Steps (re)computes global steps keep+1..end.
	newSteps := make([]integrator.StepRecord, keep, len(merged)-1)
	copy(newSteps, tr.steps[:keep])
	newSteps = append(newSteps, seg.Steps...)

	newPrefix := make([]float64, keep, len(newSteps))
	copy(newPrefix, tr.prefixMax[:keep])
	prefixMax := tr.prefixMaxAt(keep)
	for _, st := range seg.Steps {
		if st.NormDrift > prefixMax {
			prefixMax = st.NormDrift
		}
		newPrefix = append(newPrefix, prefixMax)
	}

	newBreaches := make([]breach, 0, len(tr.breaches)+len(seg.Warnings))
	for _, b := range tr.breaches {
		if b.step <= keep {
			newBreaches = append(newBreaches, b)
		}
	}
	for i, st := range seg.Steps {
		if st.NormDrift > tr.threshold {
			newBreaches = append(newBreaches, breach{step: keep + i + 1, drift: st.NormDrift})
		}
	}

	tr.samples = merged
	tr.steps = newSteps
	tr.prefixMax = newPrefix
	tr.breaches = newBreaches
	tr.seen[pkt.Seq] = packetContent{samples: copySamples(pkt.Samples)}
	tr.version++

	tip := tr.tip()
	return &AppendOutcome{
		RecomputedSteps: len(seg.Steps),
		Tip:             tip,
		Version:         tr.version,
		State:           tr.snapshot(),
	}, nil
}

// merge builds the timestamp-sorted union of the existing samples and the
// packet samples, enforcing identical content at shared timestamps, the
// retransmit window, and returning the insertion anchor (number of merged
// samples before the first packet sample) and whether any sample was new.
// It never mutates tr.samples.
func (tr *trajectory) merge(pkt Packet) (merged []integrator.Sample, anchors int, inserted bool, err error) {
	old := tr.samples
	news := pkt.Samples

	merged = make([]integrator.Sample, 0, len(old)+len(news))
	i, j := 0, 0
	anchors = -1
	for i < len(old) || j < len(news) {
		switch {
		case j >= len(news) || (i < len(old) && old[i].T < news[j].T):
			merged = append(merged, old[i])
			i++
		case i >= len(old) || (j < len(news) && news[j].T < old[i].T):
			if anchors < 0 {
				anchors = len(merged)
			}
			// Retransmission window: existing samples strictly newer than this
			// earliest inserted timestamp (zero on the very first packet).
			if newer := len(old) - i; newer > LateWindowSamples {
				return nil, 0, false, newError(validation.CauseLatePacketTooOld,
					fmtLateWindow(news[j].T, newer))
			}
			merged = append(merged, news[j])
			inserted = true
			j++
		default: // same timestamp
			if !rateEqual(old[i].W, news[j].W) {
				return nil, 0, false, newError(validation.CauseDuplicateTimestamp,
					fmtTimestampConflict(old[i].T))
			}
			merged = append(merged, old[i])
			i++
			j++
		}
	}
	if anchors < 0 {
		anchors = len(merged) // nothing new: restart point is past the end
	}
	return merged, anchors, inserted, nil
}

// prefixMaxAt returns the maximum drift among global steps 1..k (0 for k<=0
// or when fewer than k steps have been integrated so far).
func (tr *trajectory) prefixMaxAt(k int) float64 {
	if k <= 0 || k > len(tr.prefixMax) {
		return 0
	}
	return tr.prefixMax[k-1]
}

func (tr *trajectory) tip() Tip {
	t := Tip{
		SampleCount:        len(tr.samples),
		StepCount:          len(tr.steps),
		MaxNormDrift:       tr.prefixMaxAt(len(tr.steps)),
		NormDriftThreshold: tr.threshold,
		Warnings:           tr.warningTexts(),
	}
	if len(tr.samples) > 0 {
		if len(tr.steps) > 0 {
			t.Final = tr.steps[len(tr.steps)-1].Quaternion
		} else {
			t.Final = tr.q0
		}
		t.ElapsedTime = tr.samples[len(tr.samples)-1].T - tr.samples[0].T
	} else {
		t.Final = tr.q0
	}
	return t
}

// warningTexts renders the retained breaches, globally capped at the same
// number the one-shot run keeps, in the kernel's canonical wording and with
// global step indices — so the warning list equals the one-shot list.
func (tr *trajectory) warningTexts() []string {
	n := len(tr.breaches)
	if n > integrator.MaxWarnings {
		n = integrator.MaxWarnings
	}
	out := make([]string, n)
	for k := 0; k < n; k++ {
		out[k] = integrator.DriftWarning(tr.breaches[k].step, tr.breaches[k].drift, tr.threshold)
	}
	return out
}

func (tr *trajectory) snapshot() State {
	tip := tr.tip()
	st := State{
		ID:                 tr.id,
		Version:            tr.version,
		Initial:            tr.q0,
		MaxStepNormDrift:   tr.threshold,
		StrictDrift:        tr.strict,
		SampleCount:        tip.SampleCount,
		StepCount:          tip.StepCount,
		Final:              tip.Final,
		MaxNormDrift:       tip.MaxNormDrift,
		Warnings:           append([]string(nil), tip.Warnings...),
		AppliedPacketCount: len(tr.seen),
	}
	if len(tr.samples) > 0 {
		st.StartTime = tr.samples[0].T
		st.EndTime = tr.samples[len(tr.samples)-1].T
	}
	return st
}

// AttitudePoint is the result of a history query.
type AttitudePoint struct {
	T            float64
	Quaternion   quaternion.Q
	NormDrift    float64
	Interpolated bool // true when t was not itself a sample time
	StepIndex    int  // global step t belongs to (1-based); 0 at the first sample
}

// attitudeAt returns the attitude at time t within the covered range. The
// queried attitude is produced by the integration kernel itself: for an
// interior t a virtual sample is constructed with the kernel's linear
// interpolation and Advance runs the single partial step — identical to
// truncating the series at t and submitting it one-shot.
func (tr *trajectory) attitudeAt(t float64) (AttitudePoint, error) {
	if math.IsNaN(t) || math.IsInf(t, 0) {
		return AttitudePoint{}, newError(validation.CauseInvalidTimestamp,
			"查询时刻为 NaN 或无穷大")
	}
	if len(tr.samples) == 0 {
		return AttitudePoint{}, newError(validation.CauseOutOfRangeQuery,
			"轨迹尚未包含任何采样，无可查询的时刻")
	}
	t0, t1 := tr.samples[0].T, tr.samples[len(tr.samples)-1].T
	if t < t0 || t > t1 {
		return AttitudePoint{}, newError(validation.CauseOutOfRangeQuery,
			fmtOutOfRange(t, t0, t1))
	}

	// Locate the first sample with time >= t (samples are strictly increasing).
	lo, hi := 0, len(tr.samples)
	for lo < hi {
		mid := (lo + hi) / 2
		if tr.samples[mid].T < t {
			lo = mid + 1
		} else {
			hi = mid
		}
	}

	if tr.samples[lo].T == t {
		var q quaternion.Q
		if lo == 0 {
			q = tr.q0
		} else {
			q = tr.steps[lo-1].Quaternion
		}
		return AttitudePoint{T: t, Quaternion: q, Interpolated: false, StepIndex: lo}, nil
	}

	// Interior: lo is the right bracket sample, lo-1 the left.
	a, b := tr.samples[lo-1], tr.samples[lo]
	startQ := tr.q0
	if lo-1 > 0 {
		startQ = tr.steps[lo-2].Quaternion
	}
	w := integrator.LinearRateAt(a, b, t)
	seg, err := integrator.Advance(startQ,
		[]integrator.Sample{a, {T: t, W: w}},
		integrator.Options{MaxStepNormDrift: tr.threshold}, lo-1)
	if err != nil {
		return AttitudePoint{}, err
	}
	return AttitudePoint{
		T:            t,
		Quaternion:   seg.Final,
		NormDrift:    seg.MaxDrift,
		Interpolated: true,
		StepIndex:    lo,
	}, nil
}

// ---- small helpers -----------------------------------------------------------

func copySamples(in []integrator.Sample) []integrator.Sample {
	out := make([]integrator.Sample, len(in))
	copy(out, in)
	return out
}

// samplesEqual compares packet content bit-for-bit after normalizing negative
// zero, so a retransmit produced by an equivalent encoder is not a false
// conflict.
func samplesEqual(a, b []integrator.Sample) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if !sameFloat(a[i].T, b[i].T) {
			return false
		}
		if !rateEqual(a[i].W, b[i].W) {
			return false
		}
	}
	return true
}

func rateEqual(a, b [3]float64) bool {
	return sameFloat(a[0], b[0]) && sameFloat(a[1], b[1]) && sameFloat(a[2], b[2])
}

// sameFloat is exact bit equality with -0.0 normalized to +0.0. NaNs never
// occur here (validation rejects them) and are treated as unequal.
func sameFloat(a, b float64) bool {
	if a == 0 {
		a = 0
	}
	if b == 0 {
		b = 0
	}
	return a == b
}

func itoa64(i int64) string {
	if i == 0 {
		return "0"
	}
	neg := i < 0
	if neg {
		i = -i
	}
	var buf [24]byte
	p := len(buf)
	for i > 0 {
		p--
		buf[p] = byte('0' + i%10)
		i /= 10
	}
	if neg {
		p--
		buf[p] = '-'
	}
	return string(buf[p:])
}

func fmtConflict(want, got int64) string {
	return "轨迹版本不匹配: 请求基于版本 " + itoa64(want) +
		"，当前版本已为 " + itoa64(got) + "，本次追加未生效"
}

func fmtLateWindow(t float64, newer int) string {
	return "补传包最早时间戳 " + formatG(t) + " 早于最近 " + itoa64(int64(LateWindowSamples)) +
		" 个采样的覆盖范围（其后还有 " + itoa64(int64(newer)) + " 个已有采样），" +
		"超出补传窗口，拒绝整包且轨迹状态不变"
}

func fmtTimestampConflict(t float64) string {
	return "时间戳 " + formatG(t) +
		" 已存在于轨迹中但角速度值不一致，按冲突拒绝整包"
}

func fmtOutOfRange(t, t0, t1 float64) string {
	return "查询时刻 " + formatG(t) + " 超出轨迹覆盖范围 [" +
		formatG(t0) + ", " + formatG(t1) + "]"
}

func formatG(f float64) string {
	return strconv.FormatFloat(f, 'g', -1, 64)
}
