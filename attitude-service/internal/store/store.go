// Package store keeps named angular velocity series in process memory so a
// frequently used time series can be saved once and referenced repeatedly by
// the integration endpoint.
//
// The store is deliberately non-persistent: a process restart empties it,
// which is an accepted trade-off for this service. Access is serialized with
// a RWMutex so concurrent integrations referencing the same stored series
// cannot observe a partially updated value.
package store

import (
	"sort"
	"sync"

	"github.com/example/attitude-service/internal/integrator"
)

// SeriesStore is a concurrency-safe in-memory name -> series map.
type SeriesStore struct {
	mu     sync.RWMutex
	series map[string][]integrator.Sample
}

// New returns an empty store.
func New() *SeriesStore {
	return &SeriesStore{series: make(map[string][]integrator.Sample)}
}

// Save stores a defensive copy of samples under name, replacing any existing
// series of the same name.
func (s *SeriesStore) Save(name string, samples []integrator.Sample) {
	cp := make([]integrator.Sample, len(samples))
	copy(cp, samples)
	s.mu.Lock()
	s.series[name] = cp
	s.mu.Unlock()
}

// Get returns a copy of the series so callers can never mutate stored state.
func (s *SeriesStore) Get(name string) ([]integrator.Sample, bool) {
	s.mu.RLock()
	samples, ok := s.series[name]
	s.mu.RUnlock()
	if !ok {
		return nil, false
	}
	cp := make([]integrator.Sample, len(samples))
	copy(cp, samples)
	return cp, true
}

// Delete removes a series. It reports whether something was removed.
func (s *SeriesStore) Delete(name string) bool {
	s.mu.Lock()
	_, ok := s.series[name]
	delete(s.series, name)
	s.mu.Unlock()
	return ok
}

// Names lists stored series names, sorted for stable output.
func (s *SeriesStore) Names() []string {
	s.mu.RLock()
	names := make([]string, 0, len(s.series))
	for name := range s.series {
		names = append(names, name)
	}
	s.mu.RUnlock()
	sort.Strings(names)
	return names
}

// Clear empties the store (used by tests).
func (s *SeriesStore) Clear() {
	s.mu.Lock()
	s.series = make(map[string][]integrator.Sample)
	s.mu.Unlock()
}
