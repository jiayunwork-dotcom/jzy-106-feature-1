// Package trajectory implements the stateful, packet-driven attitude
// trajectory. A trajectory is opened once with an initial attitude and then
// extended packet by packet; every append continues from the current terminal
// attitude using the SAME numerical core as the one-shot integration path
// (integrator.Step), so packet boundaries can never change the result.
//
// Responsibilities kept in THIS package:
//
//   - trajectory lifecycle and the in-memory trajectory registry;
//   - per-sample archives (the attitude at every sample time) so a late packet
//     only triggers recomputation from the nearest archive before its
//     insertion point, never from the beginning;
//   - time-ordered packet merging, duplicate/conflict detection and the
//     retransmission window;
//   - trajectory versioning for optimistic concurrency.
//
// It deliberately contains no Euler-angle conversion and no HTTP code: the
// api package drives it and frames the results. All numeric integration goes
// through integrator.Step; trajectory never implements a second integration
// scheme.
package trajectory

import (
	"crypto/rand"
	"encoding/hex"
	"fmt"
	"math"
	"sort"
	"strconv"
	"sync"
	"time"

	"github.com/example/attitude-service/internal/integrator"
	"github.com/example/attitude-service/internal/quaternion"
)

// MaxLateSamples bounds how far into the past a retransmitted packet may
// reach: a late packet is accepted only when its earliest new sample is
// inserted among the most recent MaxLateSamples samples. Older retransmissions
// are rejected and leave the trajectory untouched.
const MaxLateSamples = 200

// Config is the immutable configuration fixed when a trajectory is opened.
type Config struct {
	Initial          quaternion.Q
	MaxStepNormDrift float64
	StrictDrift      bool
}

// Warning is one threshold-breach record, indexed by the GLOBAL step number
// along the whole trajectory (step i runs from sample i-1 to sample i), the
// same numbering a one-shot integration of the merged series would use.
type Warning struct {
	Step    int    `json:"step"`
	Message string `json:"message"`
}

// Machine-readable error causes. Values mirror the one-shot API where a rule
// is shared, so clients already used to "norm_drift_exceeded" need no mapping.
const (
	CauseVersionConflict      = "version_conflict"
	CausePacketConflict       = "packet_conflict"
	CauseTimestampConflict    = "timestamp_conflict"
	CauseLateOutOfWindow      = "late_packet_out_of_window"
	CauseTrajectoryNotFound   = "trajectory_not_found"
	CauseTimeOutOfRange       = "time_out_of_range"
	CauseInvalidTimestamp     = "invalid_timestamp"
	CauseNormDriftExceeded    = "norm_drift_exceeded"
	CauseTrajectoryNotOpenYet = "empty_trajectory"
)

// Error is a trajectory-level rejection with a stable cause and a Chinese,
// human-readable message. The api layer maps Cause to an HTTP status.
type Error struct {
	Cause   string
	Message string
}

func (e *Error) Error() string { return fmt.Sprintf("%s: %s", e.Cause, e.Message) }

func newError(cause, format string, args ...any) *Error {
	return &Error{Cause: cause, Message: fmt.Sprintf(format, args...)}
}

// Trajectory is one stateful attitude trajectory. Its mutex serializes every
// append/query against the same trajectory; distinct trajectories have
// distinct locks and are fully isolated.
type Trajectory struct {
	mu sync.Mutex

	id        string
	cfg       Config
	createdAt time.Time
	version   int64 // bumped once per commit that adds samples

	// Sorted samples, with the post-step attitude archived alongside:
	// atts[0] = cfg.Initial at samples[0].T; atts[i] is the attitude after
	// the step samples[i-1] -> samples[i]. drifts[i] is that step's raw
	// norm drift (drifts[0] is unused, kept zero for aligned indexing).
	samples  []integrator.Sample
	atts     []quaternion.Q
	drifts   []float64
	warns    []Warning // global-step-indexed, ordered, capped at integrator.MaxWarnings
	maxDrift float64

	// packets stores a deep copy of every accepted packet keyed by its
	// packet sequence number, for retransmission/conflict detection.
	packets map[int64][]integrator.Sample
}

// AppendResult reports the state after one append call.
type AppendResult struct {
	Version         int64
	Final           quaternion.Q
	SampleCount     int
	StepCount       int
	ElapsedTime     float64
	MaxDrift        float64
	Warnings        []Warning
	Duplicate       bool
	RecomputedSteps int // steps (re)integrated by THIS call, global step count basis
	InsertedSamples int
}

// Info is a point-in-time, fully copied view of a trajectory.
type Info struct {
	ID             string
	Version        int64
	CreatedAt      time.Time
	Initial        quaternion.Q
	Threshold      float64
	StrictDrift    bool
	SampleCount    int
	StepCount      int
	StartTime      float64
	EndTime        float64
	ElapsedTime    float64
	MaxDrift       float64
	Warnings       []Warning
	PacketSeqs     []int64
	OpenForAppends bool // false until the first packet arrives
}

// Registry is the concurrency-safe set of all live trajectories. Everything
// lives in process memory and is lost on restart, by design.
type Registry struct {
	mu sync.RWMutex
	tr map[string]*Trajectory
}

// NewRegistry returns an empty trajectory registry.
func NewRegistry() *Registry {
	return &Registry{tr: make(map[string]*Trajectory)}
}

// Create opens a new trajectory under a freshly generated ID.
func (r *Registry) Create(cfg Config) *Trajectory {
	for {
		t := &Trajectory{
			id:        newID(),
			cfg:       cfg,
			createdAt: time.Now().UTC(),
			packets:   make(map[int64][]integrator.Sample),
		}
		r.mu.Lock()
		if _, exists := r.tr[t.id]; exists {
			r.mu.Unlock()
			continue // vanishingly unlikely ID collision
		}
		r.tr[t.id] = t
		r.mu.Unlock()
		return t
	}
}

// Get looks up a trajectory by ID.
func (r *Registry) Get(id string) (*Trajectory, bool) {
	r.mu.RLock()
	t, ok := r.tr[id]
	r.mu.RUnlock()
	return t, ok
}

// Delete closes and removes a trajectory. It reports whether one existed.
func (r *Registry) Delete(id string) bool {
	r.mu.Lock()
	_, ok := r.tr[id]
	delete(r.tr, id)
	r.mu.Unlock()
	return ok
}

// IDs lists every live trajectory ID, sorted for stable output.
func (r *Registry) IDs() []string {
	r.mu.RLock()
	ids := make([]string, 0, len(r.tr))
	for id := range r.tr {
		ids = append(ids, id)
	}
	r.mu.RUnlock()
	sort.Strings(ids)
	return ids
}

// newID mints a 24-hex-char, 96-bit random identifier.
func newID() string {
	var b [12]byte
	if _, err := rand.Read(b[:]); err != nil {
		// crypto/rand failing is not a recoverable runtime condition; keep
		// the signature simple rather than plumbing a setup error through.
		panic(fmt.Errorf("trajectory: cannot generate id: %w", err))
	}
	return hex.EncodeToString(b[:])
}

// ID returns the trajectory's identifier.
func (t *Trajectory) ID() string { return t.id }

// Config returns the fixed configuration.
func (t *Trajectory) Config() Config { return t.cfg }

func samplesEqual(a, b []integrator.Sample) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i].T != b[i].T || a[i].W != b[i].W {
			return false
		}
	}
	return true
}

// Append merges one packet (already validated by the validation package) into
// the trajectory and integrates exactly the affected tail.
//
// expectedVersion, when non-nil, requests optimistic concurrency control: the
// call is rejected unless the trajectory's current version equals it. The
// whole operation runs under the trajectory lock, so concurrent appends to the
// same trajectory are strictly serialized; on ANY rejection the trajectory is
// byte-for-byte the state it was before the call.
//
// Callers must not retain or mutate packet after the call; Append copies what
// it keeps.
func (t *Trajectory) Append(seq int64, expectedVersion *int64, packet []integrator.Sample) (*AppendResult, *Error) {
	t.mu.Lock()
	defer t.mu.Unlock()

	if expectedVersion != nil && *expectedVersion != t.version {
		return nil, newError(CauseVersionConflict,
			"轨迹版本冲突: 调用方看到的版本号为 %d，当前版本号已为 %d，本次追加未施加",
			*expectedVersion, t.version)
	}

	// Retransmission / same-sequence conflict is decided before anything is
	// merged, so it can never touch the archives.
	if prev, seen := t.packets[seq]; seen {
		if samplesEqual(prev, packet) {
			return t.result(true, 0, 0), nil
		}
		return nil, newError(CausePacketConflict,
			"包序号 %d 已出现过，但本次内容与首传不一致，按冲突拒绝整包（轨迹状态不变）", seq)
	}

	merged, isNew, p0, conflictT, ok := t.merge(packet)
	if !ok {
		return nil, newError(CauseTimestampConflict,
			"时间戳 %.12g 已在轨迹采样中存在但角速度对不上，按冲突拒绝整包（轨迹状态不变）", conflictT)
	}

	newCount := 0
	for _, fresh := range isNew {
		if fresh {
			newCount++
		}
	}
	if newCount == 0 {
		// Every sample in the packet was already present with identical data:
		// an idempotent retransmission carrying a different packet seq.
		t.packets[seq] = copySamples(packet)
		return t.result(true, 0, 0), nil
	}

	// Retransmission window: the newly inserted sample must itself land among
	// the most recent MaxLateSamples samples of the resulting series, i.e. at
	// most MaxLateSamples-1 existing samples may lie behind it. An empty
	// trajectory has no "past".
	lateBehind := (len(merged) - p0) - newCount
	if lateBehind < 0 {
		lateBehind = 0
	}
	if lateBehind > MaxLateSamples-1 {
		return nil, newError(CauseLateOutOfWindow,
			"补传包最早的新采样(%.12g)早于当前末尾 %d 个采样，超出最近 %d 个采样的补传窗口，拒绝整包（轨迹状态不变）",
			merged[p0].T, lateBehind, MaxLateSamples)
	}

	// Steps actually (re)integrated: from the first affected step through the
	// new end. For the opening packet the first sample produces no step.
	recomputed := len(merged) - max(p0, 1)
	atts, drifts, warns, maxDrift, derr := t.recompute(merged, p0)
	if derr != nil {
		// Strict mode: nothing has been committed yet.
		return nil, derr
	}

	// Commit: replace the archives atomically (all locals so far).
	t.samples = merged
	t.atts = atts
	t.drifts = drifts
	t.warns = warns
	t.maxDrift = maxDrift
	t.packets[seq] = copySamples(packet)
	t.version++

	return t.result(false, newCount, recomputed), nil
}

// merge performs the timestamp-ordered two-pointer merge. It returns the
// merged samples, a per-position "comes from the new packet" flag, the index
// of the first newly inserted position, and a conflict flag carrying the
// offending timestamp when equal timestamps hold different angular rates.
func (t *Trajectory) merge(packet []integrator.Sample) (
	merged []integrator.Sample, isNew []bool, p0 int, conflictT float64, ok bool,
) {
	old := t.samples
	merged = make([]integrator.Sample, 0, len(old)+len(packet))
	isNew = make([]bool, 0, len(old)+len(packet))
	p0 = -1

	i, j := 0, 0
	for i < len(old) || j < len(packet) {
		switch {
		case j == len(packet) || (i < len(old) && old[i].T < packet[j].T):
			merged = append(merged, old[i])
			isNew = append(isNew, false)
			i++
		case i == len(old) || packet[j].T < old[i].T:
			if p0 < 0 {
				p0 = len(merged)
			}
			merged = append(merged, packet[j])
			isNew = append(isNew, true)
			j++
		default: // equal timestamps
			if old[i].W != packet[j].W {
				return nil, nil, 0, old[i].T, false
			}
			merged = append(merged, old[i])
			isNew = append(isNew, false)
			i++
			j++
		}
	}
	if p0 < 0 {
		p0 = len(merged)
	}
	return merged, isNew, p0, 0, true
}

// recompute reintegrates the tail of merged starting at position p0, reusing
// the archived attitude at p0-1. Steps strictly before p0 are untouched; the
// returned drift/warning collections are rebuilt so the global diagnostics
// equal those of a one-shot integration over the whole merged series.
func (t *Trajectory) recompute(
	merged []integrator.Sample, p0 int,
) ([]quaternion.Q, []float64, []Warning, float64, *Error) {
	n := len(merged)
	atts := make([]quaternion.Q, n)
	drifts := make([]float64, n)

	copy(atts, t.atts[:min(p0, len(t.atts))])
	copy(drifts, t.drifts[:min(p0, len(t.drifts))])

	var q quaternion.Q
	if p0 == 0 {
		q = t.cfg.Initial
		atts[0] = q // archive at the first sample time is the initial attitude
	} else {
		q = t.atts[p0-1]
	}

	// Preserve warnings for the untouched prefix, then append the first
	// threshold breaches encountered along the recomputed tail — together
	// exactly the first integrator.MaxWarnings breaches of the full series.
	warns := make([]Warning, 0, len(t.warns))
	for _, w := range t.warns {
		if w.Step < p0 {
			warns = append(warns, w)
		}
	}

	threshold := t.cfg.MaxStepNormDrift
	for i := 1; i < n; i++ {
		if i < p0 {
			continue // prefix already archived; do not redo the arithmetic
		}
		sr, err := integrator.Step(q, merged[i-1], merged[i])
		if err != nil {
			// Defensive: validation rejected non-positive steps before this.
			return nil, nil, nil, 0, newError("integration_failed",
				"第 %d 步积分失败: %v", i, err)
		}
		drifts[i] = sr.NormDrift
		if sr.NormDrift > threshold {
			if t.cfg.StrictDrift {
				return nil, nil, nil, 0, newError(CauseNormDriftExceeded,
					"step %d: quaternion norm drift %s exceeded threshold %s",
					i, formatG(sr.NormDrift), formatG(threshold))
			}
			if len(warns) < integrator.MaxWarnings {
				warns = append(warns, Warning{
					Step:    i,
					Message: integrator.DriftWarningMessage(i, sr.NormDrift, threshold),
				})
			}
		}
		nq, ok := quaternion.Normalized(sr.Raw)
		if !ok {
			return nil, nil, nil, 0, newError("integration_failed",
				"第 %d 步四元数坍缩为零，无法归一化", i)
		}
		q = nq
		atts[i] = q
	}

	// Max drift over the WHOLE trajectory, exactly what a one-shot run sees.
	maxDrift := 0.0
	for _, d := range drifts {
		if d > maxDrift {
			maxDrift = d
		}
	}
	return atts, drifts, warns, maxDrift, nil
}

// result snapshots the public state after a successful (possibly duplicate)
// append. The caller holds t.mu.
func (t *Trajectory) result(duplicate bool, inserted, recomputed int) *AppendResult {
	final := t.cfg.Initial
	if len(t.atts) > 0 {
		final = t.atts[len(t.atts)-1]
	}
	elapsed := 0.0
	if len(t.samples) > 0 {
		elapsed = t.samples[len(t.samples)-1].T - t.samples[0].T
	}
	return &AppendResult{
		Version:         t.version,
		Final:           final,
		SampleCount:     len(t.samples),
		StepCount:       max(len(t.samples)-1, 0),
		ElapsedTime:     elapsed,
		MaxDrift:        t.maxDrift,
		Warnings:        copyWarnings(t.warns),
		Duplicate:       duplicate,
		RecomputedSteps: recomputed,
		InsertedSamples: inserted,
	}
}

// Info returns a copied snapshot.
func (t *Trajectory) Info() Info {
	t.mu.Lock()
	defer t.mu.Unlock()

	seqs := make([]int64, 0, len(t.packets))
	for seq := range t.packets {
		seqs = append(seqs, seq)
	}
	sort.Slice(seqs, func(a, b int) bool { return seqs[a] < seqs[b] })

	info := Info{
		ID:             t.id,
		Version:        t.version,
		CreatedAt:      t.createdAt,
		Initial:        t.cfg.Initial,
		Threshold:      t.cfg.MaxStepNormDrift,
		StrictDrift:    t.cfg.StrictDrift,
		SampleCount:    len(t.samples),
		StepCount:      max(len(t.samples)-1, 0),
		MaxDrift:       t.maxDrift,
		Warnings:       copyWarnings(t.warns),
		PacketSeqs:     seqs,
		OpenForAppends: len(t.samples) > 0,
	}
	if len(t.samples) > 0 {
		info.StartTime = t.samples[0].T
		info.EndTime = t.samples[len(t.samples)-1].T
		info.ElapsedTime = info.EndTime - info.StartTime
	}
	return info
}

// PointResult is the attitude at one queried time.
type PointResult struct {
	T           float64
	Quaternion  quaternion.Q
	ExactSample bool // t equals an archived sample time
	BracketStep int  // global step i: atts[i] (exact) or interpolation within step i
}

// AttitudeAt returns the attitude at time t anywhere inside the covered time
// range. At a sample time the archived attitude is returned; between samples
// the attitude is advanced from the previous archive with the same linear
// interpolation and RK4 step a one-shot run would use after truncating the
// series at t and appending the interpolated sample. Out-of-range or
// non-finite times are rejected with a cause.
func (t *Trajectory) AttitudeAt(at float64) (*PointResult, *Error) {
	t.mu.Lock()
	defer t.mu.Unlock()

	if math.IsNaN(at) || math.IsInf(at, 0) {
		return nil, newError(CauseInvalidTimestamp, "查询时刻 t 为 NaN 或无穷大")
	}
	if len(t.samples) == 0 {
		return nil, newError(CauseTrajectoryNotOpenYet,
			"轨迹尚未收到任何采样，没有可查询的时间范围")
	}
	start, end := t.samples[0].T, t.samples[len(t.samples)-1].T
	if at < start || at > end {
		return nil, newError(CauseTimeOutOfRange,
			"查询时刻 %.12g 超出轨迹覆盖范围 [%.12g, %.12g]", at, start, end)
	}

	// Binary search for the first sample with T >= at.
	lo, hi := 0, len(t.samples)
	for lo < hi {
		mid := int(uint(lo+hi) >> 1)
		if t.samples[mid].T < at {
			lo = mid + 1
		} else {
			hi = mid
		}
	}

	if lo < len(t.samples) && t.samples[lo].T == at {
		return &PointResult{T: at, Quaternion: t.atts[lo], ExactSample: true, BracketStep: lo}, nil
	}

	// at lies strictly between samples[lo-1] and samples[lo].
	i := lo
	a := t.samples[i-1]
	b := t.samples[i]
	virtual := integrator.Sample{T: at, W: integrator.InterpolatedRate(a, b, at)}
	sr, err := integrator.Step(t.atts[i-1], a, virtual)
	if err != nil {
		return nil, newError("integration_failed", "插值推进失败: %v", err)
	}
	nq, ok := quaternion.Normalized(sr.Raw)
	if !ok {
		return nil, newError("integration_failed", "插值推进后四元数坍缩为零")
	}
	return &PointResult{T: at, Quaternion: nq, ExactSample: false, BracketStep: i}, nil
}

func copySamples(in []integrator.Sample) []integrator.Sample {
	out := make([]integrator.Sample, len(in))
	copy(out, in)
	return out
}

func copyWarnings(in []Warning) []Warning {
	out := make([]Warning, len(in))
	copy(out, in)
	return out
}

// formatG matches integrator's diagnostic float rendering ('g', -1).
func formatG(f float64) string {
	return strconv.FormatFloat(f, 'g', -1, 64)
}
