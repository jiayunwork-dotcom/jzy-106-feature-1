// Package trajectory implements the stateful, packet-wise continuation of an
// attitude integration.
//
// A trajectory is opened with an initial attitude and (optionally) a drift
// threshold and strict-reject policy. Afterwards angular-velocity samples
// arrive in packets, each carrying a packet sequence number; Append merges a
// packet into the trajectory and continues the integration from the last
// archived post-step attitude, reusing the integrator package's RK4 stepping
// core verbatim — this package contains no integration math of its own.
//
// Responsibilities kept HERE (deliberately separated from the kernel):
//   - in-memory trajectory state and per-sample post-step archives;
//   - ordered merge of packet samples, late retransmission insertion and the
//     bounded re-integration window;
//   - packet-sequence idempotency / conflict detection;
//   - optimistic versioning and per-trajectory serialization;
//   - rollback: every append is computed into fresh slices first, so a
//     rejected packet leaves no half-applied state behind.
//
// Quaternion conventions, Euler conventions and per-step drift rules are
// exactly those of the one-shot endpoint, because every step is produced by
// integrator.Advance.
package trajectory

import (
	"fmt"

	"github.com/example/attitude-service/internal/validation"
)

// Error is a trajectory-level rejection carrying a stable machine-readable
// cause (reusing the validation.Cause vocabulary) and a Chinese detail.
type Error struct {
	Cause   validation.Cause
	Message string
}

func (e *Error) Error() string { return fmt.Sprintf("%s: %s", e.Cause, e.Message) }

func newError(cause validation.Cause, msg string) *Error {
	return &Error{Cause: cause, Message: msg}
}
