# XGC2 Session Clock Guard

`xgc_session_clock_guard` is the fail-closed ROS1 data-plane boundary between
source-preserving VRPN topics and canonical experiment topics. It never
publishes robot commands. Runtime Sync/chrony remains a separate cross-machine
health concern; this product owns only the centralized Session time authority,
source-stamp admission, mapping, and canonical pose/twist projection.

The exact mode contract is frozen in every policy file:

| Run mode | Session authority | Mapping |
| --- | --- | --- |
| `simulation` | admitted global Gazebo `/clock` | identity (`mapped=raw`) |
| `physical` | system wall at epoch `W0`, advanced only by steady elapsed | affine-to-Session |
| `hybrid` | system wall at epoch `W0`, advanced only by steady elapsed | affine-to-Session |

No other authority/mapping pair starts. Simulation reports zero offset, drift,
and jitter while retaining the frozen source uncertainty floor. Physical and
Hybrid fit an affine source-to-Session relation. Freshness in all three modes
uses `std::chrono::steady_clock`, never system wall or ROS time.

## Routing and provenance

The node subscribes only below two fixed raw roots:

```text
/xgc/source/vrpn/simulation/<source_body>/{pose,twist}
/xgc/source/vrpn/physical/<source_body>/{pose,twist}
```

It publishes only admitted samples to:

```text
/vrpn_client_node/<canonical_body>/{pose,twist}
```

`source_body` and `canonical_body` are separate frozen fields. For example, a
physical Scout may consume mocap rigid body `ugv1` while projecting to the
Experiment namespace body `uav7`. Raw `(source_domain, source_body)` topics and
canonical bodies are each unique. Both body fields must be legal single ROS
name segments.

Experiment SlotIDs are lineage, not ROS names: values such as `px4-01` are
valid, remain in messages and the policy, and are never interpolated into a
topic. Per-route status and envelope topics use the already-validated canonical
body:

```text
/xgc/session/clock/vrpn/<canonical_body>/status
/xgc/session/clock/vrpn/<canonical_body>/envelope
```

The envelope contains both source and canonical bodies, the exact raw stamp,
mapped Session stamp, steady receipt, authority age, Gazebo skew, epoch,
`run_mode`, mapping diagnostics, acceptance state, and canonical publish
outcome. Route status also carries `run_mode`; aggregate status additionally
carries a process-local, strictly increasing `status_sequence` used by the
live readiness proof.

## Gazebo and station authority gates

Simulation consumes global `/clock`. Hybrid consumes the private
`/xgc/source/gazebo/clock`; physical mode has no Gazebo clock subscriber. A
Gazebo authority must become positive, move strictly forward, remain fresh by
steady-clock age, and stay within the frozen source-stamp skew. Hybrid also
requires two accepted private-clock samples and keeps the measured Gazebo
real-time factor within frozen bounds that strictly bracket `1.0`.

This ROS1 node cannot securely prove graph publisher identity or uniqueness.
Core workflow admission must prove the one expected Gazebo publisher before
starting the guard and must retain the matching resource claim. The product
does not pretend topic receipt is that proof; it enforces the timestamp,
freshness, skew, and RTF gates after external admission.

Physical and Hybrid freeze `(W0, M0)` at epoch start and derive:

```text
SessionNow = W0 + (steady_now - M0)
wall_error = current_system_wall - SessionNow
wall_step  = wall_error - previous_wall_error
```

System wall is monitoring input after epoch start, not a re-anchor source. A
wall error or wall step outside its frozen threshold immediately enters
`lost`. The same epoch can never be re-anchored; only a strictly greater epoch
can recover it.

## Frozen configuration and ownership

Startup requires three trusted private parameters with no defaults:

- `policy_file`: immutable Session-generated v2 config;
- `policy_sha256`: exact lowercase SHA-256 of its bytes;
- `epoch_id`: canonical nonzero decimal Session epoch allocated for this new
  process by the Core Session lifecycle.

The direct-executable Process definition wraps `${epochId}` in exactly one
pair of ASCII double quotes inside the ROS private-parameter argument. This is
required because roscpp otherwise coerces small decimals to XMLRPC integers
and large decimals to lossy doubles. The node unwraps only that exact form;
the launch-file path may still provide a raw string. XMLRPC integers/doubles,
single-quoted values, YAML tags, whitespace, leading zeroes, and overflow all
fail closed.

The config is a canonical flat `key=value` document. Unknown, duplicated,
missing, reordered, whitespace-padded, malformed, or mode-mismatched fields
fail startup. The fixed Parameter Asset may supply only policy thresholds, IO
queue depth, and trusted per-product sampling constants. Session/Core must
generate `session_id`, contract digest, run mode, authority/mapping pair, and
every route. `epoch_id` is deliberately not a config key. The product consumes
only the final digest-verified bytes; the Core materializer is responsible for
proving that authors cannot override Session-owned fields.

The Guard derives both `epoch-state.json` and `epoch-state.lock` from the
parent directory of `policy_file`; there is no fourth parameter and no
author-selectable state or lock path. Core creates the immutable policy, state,
and empty lock file in the same private Session directory. Both runtime files
must be regular, non-symlink files with exact mode `0600`, and the lock must
remain empty; the state contains canonical compact JSON and one trailing LF.
Its schema is
`xgc.session-clock-policy.epoch-state.v1`, with exactly these fields in Core's
canonical order:

```text
epochId, jobId, policySha256, schema, sessionContractSha256, sessionId, targetId
```

The state epoch, Session, Session-contract digest, and policy digest must match
the startup tuple. Before reading it, the Guard takes a nonblocking shared
`flock` on the lock inode and holds that lease until process exit. Core must
take `LOCK_EX|LOCK_NB` on the same inode before any legal state create/replace,
so epoch allocation explicitly fails while an old Guard is alive. The node
also checks that the path still names the held inode and rereads state before
and after construction and continuously while polling and receiving inputs.
The health check holds a shared lease and validates it before and after live
ROS evidence. Missing, replaced, noncanonical, over-permissive, or
out-of-band-mutated state hard-loses the Guard, closes canonical output, and
makes that process exit.

The initial fixed per-product timing constants are:

| Robot kind | `sample_period_ns` | `source_uncertainty_ns` |
| --- | ---: | ---: |
| PX4 / FS150 | `8333333` | `5000000` |
| Scout | `10000000` | `6000000` |
| Mecanum | `10000000` | `6000000` |

These are materializer inputs, not defaults in the Guard. Before formal field
use, CI/live evidence must confirm that the packaged source actually publishes
at the declared cadence; otherwise the fixed Parameter Asset must be revised
and repinned rather than silently changing the generated policy.

The common frozen `threshold.startup_lock_timeout_ns` is separate from runtime
freshness. It is accepted only in `[250000000, 3000000000]`, cannot be below
either runtime age bound, and is `3000000000` in the installed examples. From
the epoch anchor it bounds only the first positive Gazebo clock (when required)
and the first pose and twist sample of every route. A missing required source
past that timeout is a hard `lost` transition and requires a new Core-started
process at a greater persisted epoch. As soon as an individual clock or stream
has been seen, its existing `max_authority_age_ns` or `max_sample_age_ns` applies;
the startup allowance never weakens runtime stale detection.

Pure Simulation and Physical policies may contain zero Robot routes for
camera-only intrinsic/extrinsic calibration. In that case the guard still owns
the epoch, authority gates, aggregate status, and events; it never fabricates a
Robot route or canonical pose topic. Simulation locks after its admitted
positive `/clock`; Physical locks on the immutable station anchors. Hybrid
always requires at least one Simulation and one Physical route.

Gazebo VRPN v24 carries `timeval` at 1 microsecond resolution, so
`vrpn.wire_time_resolution_ns` must be exactly `1000`. Every route's source
uncertainty must cover that quantization plus half its declared sample period.
`delay.timestamp_policy` must be exactly `sample_time`; `send_time` and spelling
variants fail closed in every mode.

Three non-production examples and their exact digests are installed in
`config/`:

```bash
cd "$(rospack find xgc_session_clock_guard)/config"
sha256sum --check \
  example-simulation-v24.cfg.sha256 \
  example-physical-v24.cfg.sha256 \
  example-hybrid-v24.cfg.sha256
```

These files demonstrate canonical syntax and digest verification only. They
are not directly launchable experiment artifacts: they have no trusted
Core-created sibling state and lock files. Hand-written runtime files or a
manual `roslaunch` are not a supported replacement for the dedicated Core
Session workflow.

## State, events, and recovery

- `initializing`: authority/routes are collecting evidence; output is closed.
- `locked`: every authority and route gate holds; canonical output may open.
- `degraded`: a bounded violation occurred; output closes immediately.
- `lost`: a hard violation or the frozen failure count was reached; samples
  cannot relock this epoch.

Both pose and twist timelines are polled by steady time, so total silence or one
missing stream closes output. Gazebo rollback and station wall error/step are
hard loss. Missing initial authority/streams past the frozen startup timeout are
also hard loss. Gazebo stagnation, runtime stale authority, skew, RTF, and
stream silence degrade immediately and reach loss at the frozen consecutive-
failure count.

`/xgc/session/clock/events` emits `new_epoch`, `locked`, `degraded`, and `lost`
transitions with sequence, epoch, authority age, Gazebo stamp/skew/RTF, station
wall error/step, steady timestamp, Session timestamp, and reason. Aggregate
status remains latched at `/xgc/session/clock/status`, but a latched sample by
itself is never readiness.

There is no global epoch-advance service and a running Guard never changes its
epoch. After `lost`, Core must first stop and confirm exit of the old Guard,
then acquire the exclusive lock, advance persistent epoch state with a distinct
Job identity, release it, and construct a new Guard. A concurrent materializer
cannot use epoch advance as a way to evict a live Guard. The Guard never steps
a clock, silently selects a fallback source, or reuses an epoch anchor.

## Development and remaining live gates

ROS-free logic and static product contracts run on a host without ROS:

```bash
tests/run_core_tests.sh
tests/static_contract_test.sh
```

Formal delivery is Ubuntu 20.04 Focal, ROS Noetic, amd64 and arm64. The package
installs its Process definition under `/usr/share/xgc2/process-definitions` and
owns canonical/sidecar roots exclusively while sharing raw source roots.
The definition is `internal:true`, so generic author process creation cannot
start it. The dedicated trusted Session runner selects the current internal
definition and pins its digest, then lowers it through Core's supervised
generic runtime only after the internal-definition gate has been explicitly
opened in-process. No workflow-authored parameters can exercise that gate.

Readiness is an installed exec probe, not a generic topic-flow probe. It first
asks the ROS master to prove that `/xgc/session/clock/status` has exactly one
publisher named `/xgc_session_clock_guard`. It then requires two messages from
that same publisher with strictly advancing `status_sequence`; an external
publisher, changed identity, or a strictly lower sequence fails closed. One
latched message, or same-sequence repeats through the deadline, never satisfies
readiness; a duplicate followed by a greater sequence may continue. Both
messages must match Session/contract/policy identities, epoch, `run_mode`,
authority/mapping, route counts, v24 fields, and all mode-specific authority,
freshness, skew, wall, and RTF thresholds. The epoch fence and ROS-master
publisher set are rechecked after the two live samples. Canonical policy
validation rejects `threshold.guard_poll_period_ns` above `250000000 ns`; the
two-second live-sample deadline therefore spans at least eight poll periods,
including a zero-route Physical calibration, and the Process exec timeout is
five seconds to leave ROS-master and secure-file-check margin. Supervisor
liveness reruns this same strong, tuple-bound healthcheck every second and
fails on its first unsuccessful result. A Guard that remains registered but
has entered `degraded`/`lost`, lost authority, or lost its epoch fence is
therefore not treated as live merely because the ROS node still exists; the
data plane closes within the 250 ms guard bound and Core observes probe failure
on its subsequent supervision cycle.

Source and package tests are not live acceptance. Remaining live gates require
a published/installed v24 Gazebo VRPN package using `World::SimTime` and
`sample_time`, an admitted global Simulation or private Hybrid Gazebo clock
publisher, a physical VRPN source, ROS master traffic on both raw roots, wall
step/authority-stale fault injection, and packet/status/event evidence that
canonical output closes. They must also prove that a live Guard blocks Core's
nonblocking exclusive allocation, that stop/confirmed-exit permits a greater
persisted epoch, and that only the newly constructed Guard at that epoch can
lock again.
