package trajectory

import (
	"crypto/rand"
	"encoding/hex"
	"sort"
	"sync"
	"time"

	"github.com/example/attitude-service/internal/validation"
)

// Manager owns every live trajectory. State is in-process memory only; a
// restart empties it, which is an accepted trade-off.
//
// Concurrency:
//   - the manager mutex guards the id -> trajectory map;
//   - each trajectory has its own mutex serializing appends/queries on THAT
//     trajectory, so concurrent appends can never step from the same stale
//     end attitude;
//   - distinct trajectories are fully isolated and may proceed in parallel.
type Manager struct {
	mu sync.Mutex
	ts map[string]*trajectory
}

// NewManager returns an empty trajectory manager.
func NewManager() *Manager {
	return &Manager{ts: make(map[string]*trajectory)}
}

// Open creates a trajectory from a validated config and returns its ID and
// initial state (version 0, no samples yet).
func (m *Manager) Open(cfg Config) (string, State, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	for {
		id, err := newID()
		if err != nil {
			return "", State{}, err
		}
		if _, exists := m.ts[id]; exists {
			continue // astronomically unlikely; draw a fresh ID
		}
		tr := newTrajectory(id, cfg, time.Now().Unix())
		m.ts[id] = tr
		return id, State{
			ID:               id,
			Version:          0,
			Initial:          cfg.Q0,
			MaxStepNormDrift: tr.threshold,
			StrictDrift:      cfg.StrictDrift,
			Final:            cfg.Q0,
			Warnings:         []string{},
		}, nil
	}
}

// Get returns a defensive status snapshot of one trajectory.
func (m *Manager) Get(id string) (State, bool) {
	tr, ok := m.lookup(id)
	if !ok {
		return State{}, false
	}
	tr.mu.Lock()
	defer tr.mu.Unlock()
	return tr.snapshot(), true
}

// List returns status snapshots of every trajectory, sorted by ID.
func (m *Manager) List() []State {
	m.mu.Lock()
	ids := make([]string, 0, len(m.ts))
	for id := range m.ts {
		ids = append(ids, id)
	}
	m.mu.Unlock()
	sort.Strings(ids)

	out := make([]State, 0, len(ids))
	for _, id := range ids {
		if st, ok := m.Get(id); ok {
			out = append(out, st)
		}
	}
	return out
}

// Append applies one packet to one trajectory, serializing concurrent
// appends to the same trajectory. expectedVersion < 0 skips the optimistic
// version check.
func (m *Manager) Append(id string, pkt Packet, expectedVersion int64) (*AppendOutcome, error) {
	tr, ok := m.lookup(id)
	if !ok {
		return nil, newError(validation.CauseTrajectoryNotFound,
			"轨迹不存在或已关闭: "+id)
	}
	tr.mu.Lock()
	defer tr.mu.Unlock()
	return tr.append(pkt, expectedVersion)
}

// AttitudeAt answers a history query for one trajectory.
func (m *Manager) AttitudeAt(id string, t float64) (AttitudePoint, error) {
	tr, ok := m.lookup(id)
	if !ok {
		return AttitudePoint{}, newError(validation.CauseTrajectoryNotFound,
			"轨迹不存在或已关闭: "+id)
	}
	tr.mu.Lock()
	defer tr.mu.Unlock()
	return tr.attitudeAt(t)
}

// Close deletes a trajectory and its archives. It reports whether it existed.
func (m *Manager) Close(id string) bool {
	m.mu.Lock()
	tr, ok := m.ts[id]
	if ok {
		delete(m.ts, id)
	}
	m.mu.Unlock()
	if !ok {
		return false
	}
	// Wait out any in-flight append on this trajectory so its effects cannot
	// resurrect state after close.
	tr.mu.Lock()
	tr.mu.Unlock()
	return true
}

func (m *Manager) lookup(id string) (*trajectory, bool) {
	m.mu.Lock()
	tr, ok := m.ts[id]
	m.mu.Unlock()
	return tr, ok
}

// newID returns a 128-bit random hex identifier.
func newID() (string, error) {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return "", err
	}
	return hex.EncodeToString(b[:]), nil
}
