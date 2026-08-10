#!/usr/bin/env python3
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any


def no_duplicate_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


root = Path(__file__).resolve().parents[1]
catalog = root / "runtime-manifests/process-definitions"
manifests = sorted(catalog.rglob("*.json"))
if manifests != [catalog / "xgc2-session-clock-guard.json"]:
    raise AssertionError(f"unexpected discovered Process manifests: {manifests}")

definition_plugin = json.loads(
    manifests[0].read_text(encoding="utf-8"), object_pairs_hook=no_duplicate_object
)
if set(definition_plugin) != {"apiVersion", "definitions"}:
    raise AssertionError("Process plugin top-level contract is not closed")
if definition_plugin["apiVersion"] != "xgc.execution.process/v1":
    raise AssertionError("unsupported Process plugin apiVersion")
if len(definition_plugin["definitions"]) != 1:
    raise AssertionError("product must discover exactly one Process definition")

definition = definition_plugin["definitions"][0]
if definition["id"] != "xgc2-session-clock-guard" or definition["version"] != "0.1.0":
    raise AssertionError("Process identity/version mismatch")
if definition["drivers"] != ["host"]:
    raise AssertionError("Session Clock Guard must remain Core-local host-only")
if definition.get("internal") is not True:
    raise AssertionError(
        "Session Clock Guard must be internal and unavailable to generic author process creation"
    )
if definition.get("parameterPolicyRevision") != "core/session-clock-guard/v1":
    raise AssertionError("Session Clock Guard must declare the registered Core policy revision")

schema = definition["parameters"]
properties = schema["properties"]
if schema["additionalProperties"] is not False:
    raise AssertionError("Process parameter schema must be closed")
if set(schema["required"]) != {"policyFile", "policySha256", "epochId"}:
    raise AssertionError("policy file, digest, and epoch must be the exact required inputs")
for required in schema["required"]:
    if "default" in properties[required]:
        raise AssertionError(f"required frozen input {required} must not have a default")
for name in properties:
    if name.startswith("threshold") or name.startswith("max") or name.endswith("Ns"):
        raise AssertionError(
            f"workflow-visible threshold {name} would bypass the hashed policy file"
        )
if properties["policyFile"].get("x-xgc-path-kind") != "file":
    raise AssertionError("policyFile must use the trusted file-path contract")
if properties["rosNodeName"].get("default") != "xgc_session_clock_guard":
    raise AssertionError("rosNodeName must be a slash-free roscpp __name value")

command = definition["command"]
if command.get("directExecutable") is not True:
    raise AssertionError("Process must use direct argv execution")
if command["executable"] != (
    "${rosInstallPath}/lib/xgc_session_clock_guard/session_clock_guard_node"
):
    raise AssertionError("Process executable is not the APT-installed ROS node")
if command["args"] != [
    "__name:=${rosNodeName}",
    "_policy_file:=${policyFile}",
    "_policy_sha256:=${policySha256}",
    '_epoch_id:="${epochId}"',
]:
    raise AssertionError("Process argv does not bind the exact frozen inputs")

tokens = set(re.findall(r"\$\{([A-Za-z0-9_.-]+)\}", json.dumps(command)))
if not tokens.issubset(properties):
    raise AssertionError(f"command references undeclared parameters: {tokens - set(properties)}")

claims = {claim["bindingKey"]: claim for claim in definition["resourceClaims"]}
expected_claims = {
    "canonical-vrpn-root",
    "physical-raw-vrpn-root",
    "ros-node",
    "session-clock-sidecar-root",
    "simulation-raw-vrpn-root",
}
if set(claims) != expected_claims:
    raise AssertionError("Process resource ownership set is incomplete or expanded")


def claim_literal(name: str) -> str:
    parts = claims[name]["identityParts"]
    if parts[0] != {"parameter": "rosMasterUri"} or "literal" not in parts[1]:
        raise AssertionError(f"claim {name} is not anchored to the ROS master and fixed root")
    return parts[1]["literal"]


if claim_literal("simulation-raw-vrpn-root") != "/xgc/source/vrpn/simulation":
    raise AssertionError("simulation raw root mismatch")
if claim_literal("physical-raw-vrpn-root") != "/xgc/source/vrpn/physical":
    raise AssertionError("physical raw root mismatch")
for raw in ("simulation-raw-vrpn-root", "physical-raw-vrpn-root"):
    if claims[raw]["mode"] != "shared" or claims[raw]["capacity"] < 1:
        raise AssertionError(f"raw subscriber claim {raw} must be shared")
if claim_literal("canonical-vrpn-root") != "/vrpn_client_node":
    raise AssertionError("canonical VRPN root mismatch")
if claims["canonical-vrpn-root"]["mode"] != "exclusive":
    raise AssertionError("canonical VRPN root must have one owner")
if claim_literal("session-clock-sidecar-root") != "/xgc/session/clock":
    raise AssertionError("Session clock sidecar root mismatch")
if claims["session-clock-sidecar-root"]["mode"] != "exclusive":
    raise AssertionError("Session clock sidecar root must have one owner")

readiness = definition["readiness"]
if readiness.get("kind") != "exec":
    raise AssertionError("readiness must validate locked admission, not only topic flow")
readiness_command = readiness.get("command", {})
if readiness_command.get("executable") != (
    "${rosInstallPath}/lib/xgc_session_clock_guard/session_clock_guard_healthcheck"
):
    raise AssertionError("readiness does not use the installed locked-state checker")
if readiness_command.get("args") != ["${policyFile}", "${policySha256}", "${epochId}"]:
    raise AssertionError("readiness checker is not bound to frozen policy and epoch")
if readiness.get("timeout") != 5_000_000_000:
    raise AssertionError("readiness timeout must leave margin around live ROS evidence")
liveness = definition["liveness"]
if liveness.get("kind") != "exec":
    raise AssertionError("liveness must rerun strong locked admission")
if liveness.get("command") != readiness_command:
    raise AssertionError("liveness must use the same installed bound healthcheck")
if (
    liveness.get("interval") != 1_000_000_000
    or liveness.get("timeout") != 5_000_000_000
    or liveness.get("successThreshold") != 1
    or liveness.get("failureThreshold") != 1
):
    raise AssertionError("liveness cadence must fail the Session promptly")

encoded = json.dumps(definition).lower()
for forbidden in ("/cmd_vel", "setpoint", "arming", "takeoff", "land"):
    if forbidden in encoded:
        raise AssertionError(f"motion-command surface leaked into Process definition: {forbidden}")

print("session-clock-guard Process discovery contract passed")
