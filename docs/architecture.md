# Session clock and routing contract

## Boundaries

This product owns centralized ROS1 time admission, mapping, and canonical
pose/twist projection. It does not own Gazebo, VRPN transport, graph publisher
identity admission, chrony, robot drivers, `swarm_ros_bridge`, MAVROS,
workflow orchestration, recording, or motion control.

Core workflow must admit exactly one expected publisher for global Simulation
`/clock` or private Hybrid `/xgc/source/gazebo/clock`. The guard starts from a
digest-pinned policy and a Core-owned epoch fence and enforces the observable
clock contract after that external graph admission.

## Core-owned process and epoch boundary

The Process definition is `internal:true`. Generic author process creation
must reject it. A dedicated Session runner selects the current definition,
pins its digest, supplies only the trusted `policyFile`, `policySha256`, and
`epochId` tuple, and then uses the generic supervised runtime behind Core's
explicit in-process internal gate. This preserves one runtime implementation
without exposing internal creation to workflow authors.

`epochId` is constructor-frozen for one process. There is no ROS service or
other runtime surface that can advance it. Core's materializer stores the
immutable policy, `epoch-state.json`, and `epoch-state.lock` together in a
private Session directory. The Guard derives both runtime paths from
`policyFile`; neither is an additional Process parameter.

The state is a regular, non-symlink `0600` file containing canonical compact
JSON plus LF under schema `xgc.session-clock-policy.epoch-state.v1`. Its exact
Core field order is `epochId`, `jobId`, `policySha256`, `schema`,
`sessionContractSha256`, `sessionId`, `targetId`. The lock is also a fixed,
empty, regular, non-symlink `0600` file. The Guard holds
`flock(LOCK_SH|LOCK_NB)` on that exact inode for its lifetime; Core must obtain
`LOCK_EX|LOCK_NB` before
legal state creation or replacement. Thus a live Guard makes new epoch
allocation fail instead of racing a canonical publish. Continuous same-inode,
file-shape, and lineage validation still fails closed on out-of-band mutation.
Recovery stops and confirms exit of the old Guard before Core exclusively
allocates and starts the next epoch.

## Three clocks, two authorities

The implementation keeps three clock concepts separate:

- Gazebo clock: source-domain authority in Simulation and a private RTF/skew
  witness in Hybrid.
- system wall: sampled once as `W0` in Physical/Hybrid, then monitored only for
  error and steps.
- steady clock: monotonic elapsed-time and freshness source in all modes.

Simulation uses Gazebo as Session authority and maps VRPN sample stamps by
identity. Physical/Hybrid derive Session time from immutable epoch anchors:

```text
SessionNow(M) = W0 + (M - M0)
```

Live wall time cannot update `W0`. Its difference from derived Session time and
the delta of that difference are independently bounded. Either wall threshold
is a hard `lost` transition, so recovery requires a greater epoch and new
anchors.

## Gazebo gates

The Gazebo input must be positive, strictly monotonic, and fresh by steady age.
Every Simulation-domain VRPN observation must be within the frozen skew of the
latest admitted Gazebo clock. Simulation consumes global `/clock` and returns
the raw stamp unchanged with zero offset/drift/jitter.

Hybrid consumes a private Gazebo clock. Its first positive sample establishes
lineage but does not make authority ready; the second and later samples must
establish a real-time factor inside frozen bounds that strictly bracket `1.0`.
Hybrid Simulation routes then pass both private-clock freshness/skew and affine
mapping into station Session time. Physical routes share the same station
Session time but do not receive a Gazebo skew test.

The first required Gazebo sample uses the common frozen startup-lock timeout,
measured from the epoch steady anchor. That timeout is distinct from runtime
authority freshness: after the first sample, `max_authority_age_ns` applies
immediately. Missing the startup timeout is a hard loss and cannot be repaired
within the same epoch.

## Affine estimator and steady freshness

Physical and Hybrid routes fit:

```text
session_time = source_time + offset(source_time)
offset(source_time) = intercept + drift * (source_time - anchor)
```

Jitter is the maximum absolute regression residual in the frozen window.
Uncertainty is the saturating sum of the route's frozen source floor and
jitter. An underfilled affine window collects observations while canonical
output remains closed; provisional estimator quality is not treated as a
failure. The candidate that reaches `min_lock_samples` immediately gates
drift, jitter, uncertainty, and offset step before it can be committed.
Prediction age is enforced once a complete prior affine fit has been committed,
and every observation after lock remains subject to the full frozen gates.

Pose and twist have independent source and receipt timelines because one VRPN
sample may stamp both streams identically. A VRPN producer may also emit an
exact same-stream duplicate at one Gazebo step; the Guard drops that duplicate
without counting freshness or a healthy sample. A strictly lower same-stream
stamp remains a hard epoch loss. Receipt and silence age use only the last
accepted unique sample and steady time, so replaying duplicates cannot keep a
silent route alive. A wall timer polls both timelines, preventing a silent
route from remaining locked.
Each unseen pose/twist timeline initially uses the same startup-lock timeout;
once that individual timeline is seen, it immediately uses
`max_sample_age_ns`. Thus ROS graph connection startup has a bounded admission
window without widening runtime silence detection.

## Route identity

Each route freezes independent `source_body` and `canonical_body` fields. Raw
topic uniqueness is checked on `(source_domain, source_body)`; canonical body
uniqueness prevents two routes from contending for one canonical topic. Both
are legal single ROS segments.

SlotID is only contract lineage and may contain `-`. Sidecar status/envelope
topics therefore use `canonical_body`, not SlotID. Messages preserve SlotID,
source body, and canonical body together for audit and replay.

## Aggregate admission and events

A route may lock locally, but canonical publish remains false until authority
and every frozen route are locked. One initializing, degraded, or lost route
closes the entire Session route set. This prevents partial Hybrid admission.

Transition events record new epoch, aggregate lock, degradation, and loss with
both clock-domain diagnostics. Events complement, rather than replace, latched
route/aggregate status and the per-observation timestamp envelope.

Pure Simulation/Physical camera-calibration Sessions may freeze zero Robot
routes. Their aggregate state follows authority alone, including lock and loss
events. Hybrid still requires both route domains.

Process readiness invokes the installed digest/epoch-bound health checker. It
requires the ROS master to report exactly one
`/xgc/session/clock/status` publisher named `/xgc_session_clock_guard`, then
receives at least two messages from that same publisher with strictly advancing
`status_sequence`. Each must prove the frozen identities, `run_mode`, authority
and mapping, locked aggregate/authority state, route closure, v24 fields, and
the applicable freshness, Gazebo skew, station wall, and Hybrid RTF bounds.
The checker validates the epoch fence before and after ROS evidence. A single
latched sample or same-sequence repeats through the deadline are not readiness;
a lower sequence, external publisher, changed identity, or stale fence is an
immediate rejection. Canonical policy rejects a guard poll period above
`250000000 ns`. The two-second sample deadline covers at least eight poll
periods, including authority-only zero-route Sessions; the Process gives the
exec probe five seconds total for ROS-master and secure-file checks. Liveness
uses the identical installed checker every second with failure threshold one;
node registration alone cannot keep a degraded/lost or stale-fence Session
active.

The product emits no `cmd_vel`, MAVROS setpoint, arm/disarm, takeoff, land, or
other actuation topic. Downstream control must treat aggregate lock as a
prerequisite, never as an instruction to move.
