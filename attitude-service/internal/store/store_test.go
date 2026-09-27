package store_test

import (
	"testing"

	"github.com/example/attitude-service/internal/integrator"
	"github.com/example/attitude-service/internal/store"
)

func TestSaveGetDelete(t *testing.T) {
	s := store.New()
	samples := []integrator.Sample{
		{T: 0, W: [3]float64{0, 0, 1}},
		{T: 0.1, W: [3]float64{0, 0, 1}},
	}
	s.Save("a", samples)
	got, ok := s.Get("a")
	if !ok {
		t.Fatal("saved series not found")
	}
	if len(got) != 2 || got[1].T != 0.1 {
		t.Fatalf("stored series mismatch: %+v", got)
	}

	names := s.Names()
	if len(names) != 1 || names[0] != "a" {
		t.Fatalf("names = %v", names)
	}

	if !s.Delete("a") {
		t.Fatal("delete returned false for existing key")
	}
	if s.Delete("a") {
		t.Fatal("delete returned true for missing key")
	}
	if _, ok := s.Get("a"); ok {
		t.Fatal("deleted series still retrievable")
	}
}

// TestStoredSamplesAreCopied verifies that neither saving a series nor reading
// one back lets the caller mutate stored state — a prerequisite for safely
// reusing a named series from many concurrent integrations.
func TestStoredSamplesAreCopied(t *testing.T) {
	s := store.New()
	in := []integrator.Sample{
		{T: 0, W: [3]float64{0, 0, 1}},
		{T: 0.1, W: [3]float64{0, 0, 1}},
	}
	s.Save("a", in)

	// Mutate the slice that was saved.
	in[0].T = 999
	got, _ := s.Get("a")
	if got[0].T != 0 {
		t.Fatalf("store aliased the save-time slice: %+v", got)
	}

	// Mutate the slice that was returned.
	got[1].W[0] = 123
	got2, _ := s.Get("a")
	if got2[1].W[0] != 0 {
		t.Fatalf("store aliased the returned slice: %+v", got2)
	}
}
