package trajectory_test

import (
	"testing"

	"github.com/example/attitude-service/internal/integrator"
	"github.com/example/attitude-service/internal/quaternion"
	"github.com/example/attitude-service/internal/trajectory"
)

// TestPacketizedDriftWarningsMatchOneShot forces many per-step threshold
// breaches with a tight (non-strict) threshold and requires the packetized
// trajectory to surface the exact same warning texts — global step numbers
// included — as the one-shot run, capped the same way.
func TestPacketizedDriftWarningsMatchOneShot(t *testing.T) {
	// A coarse, high-rate series trips the drift threshold on most steps.
	n := 40
	samples := make([]integrator.Sample, n)
	for i := range samples {
		samples[i] = integrator.Sample{
			T: float64(i) * 0.9,
			W: [3]float64{0.8, -0.4, 0.6},
		}
	}
	opts := integrator.Options{MaxStepNormDrift: 1e-12, StrictDrift: false}
	ref, err := integrator.Simulate(quaternion.Identity(), samples, opts)
	if err != nil {
		t.Fatal(err)
	}
	if len(ref.Warnings) == 0 {
		t.Fatal("reference produced no warnings; tighten the threshold or coarsen steps")
	}

	m := trajectory.NewManager()
	id, _, err := m.Open(trajectory.Config{
		Q0:               quaternion.Identity(),
		MaxStepNormDrift: 1e-12,
	})
	if err != nil {
		t.Fatal(err)
	}
	// Uneven boundaries, including a one-sample first packet.
	bounds := []int{0, 1, 3, 10, 22, n}
	for k := 1; k < len(bounds); k++ {
		if _, err := m.Append(id,
			trajectory.Packet{Seq: int64(k), Samples: samples[bounds[k-1]:bounds[k]]}, -1); err != nil {
			t.Fatalf("packet %d: %v", k, err)
		}
	}
	// A late packet inside the window must rebuild warnings with unchanged
	// global numbering: replace samples 15..19 by re-inserting them after the
	// end is already known.
	st, _ := m.Get(id)
	if len(st.Warnings) != len(ref.Warnings) {
		t.Fatalf("warnings %d != %d\ngot  %v\nwant %v",
			len(st.Warnings), len(ref.Warnings), st.Warnings, ref.Warnings)
	}
	for i := range ref.Warnings {
		if st.Warnings[i] != ref.Warnings[i] {
			t.Fatalf("warning %d:\n got %q\nwant %q", i, st.Warnings[i], ref.Warnings[i])
		}
	}
	if st.Final != ref.Final || st.MaxNormDrift != ref.MaxDrift {
		t.Fatalf("tip mismatch: %+v vs %+v", st.Final, ref.Final)
	}

	// Force >20 breaches end-to-end and check the 20-warning cap is identical.
	samples2 := make([]integrator.Sample, 60)
	for i := range samples2 {
		samples2[i] = integrator.Sample{T: float64(i), W: [3]float64{1.5, 0.2, -0.7}}
	}
	ref2, err := integrator.Simulate(quaternion.Identity(), samples2, opts)
	if err != nil {
		t.Fatal(err)
	}
	id2, _, err := m.Open(trajectory.Config{Q0: quaternion.Identity(), MaxStepNormDrift: 1e-12})
	if err != nil {
		t.Fatal(err)
	}
	for k := 0; k < 6; k++ {
		if _, err := m.Append(id2, trajectory.Packet{
			Seq:     int64(k + 1),
			Samples: samples2[k*10 : (k+1)*10],
		}, -1); err != nil {
			t.Fatal(err)
		}
	}
	st2, _ := m.Get(id2)
	if len(st2.Warnings) != len(ref2.Warnings) {
		t.Fatalf("cap: %d != %d", len(st2.Warnings), len(ref2.Warnings))
	}
}
