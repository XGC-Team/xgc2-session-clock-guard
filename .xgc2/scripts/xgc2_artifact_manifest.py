#!/usr/bin/env python3
"""Create and verify Session Clock Guard trusted build manifests."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any


BUILD_SCHEMA = "xgc2.build-artifact.v1"
BUILD_FIELDS = {
    "schema",
    "product",
    "version",
    "source_sha",
    "distribution",
    "architecture",
    "ci",
    "debs",
}
CI_FIELDS = {"run_id", "workflow", "workflow_ref"}
DEB_FIELDS = {"file", "package", "version", "architecture", "sha256", "size"}
SOURCE_SHA = re.compile(r"^[0-9a-f]{40}(?:[0-9a-f]{24})?$")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def deb_metadata(path: Path) -> dict[str, Any]:
    result = subprocess.run(
        [
            "dpkg-deb",
            "--show",
            "--showformat=${Package}\n${Version}\n${Architecture}\n",
            str(path),
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    fields = result.stdout.splitlines()
    if len(fields) != 3 or not all(fields):
        raise ValueError(f"invalid Debian metadata: {path}")
    package, version, architecture = fields
    return {
        "file": path.name,
        "package": package,
        "version": version,
        "architecture": architecture,
        "sha256": sha256(path),
        "size": path.stat().st_size,
    }


def validate_identity(args: argparse.Namespace) -> None:
    if args.product != "xgc2-session-clock-guard":
        raise ValueError("product identity is not xgc2-session-clock-guard")
    if args.expected_package != "ros-noetic-xgc2-session-clock-guard":
        raise ValueError("expected package identity is invalid")
    if args.distribution != "focal":
        raise ValueError("distribution must be focal")
    if args.architecture not in {"amd64", "arm64"}:
        raise ValueError("architecture must be amd64 or arm64")
    if SOURCE_SHA.fullmatch(args.source_sha) is None:
        raise ValueError("source SHA must be 40 or 64 lowercase hexadecimal characters")


def validated_deb(args: argparse.Namespace, path: Path) -> dict[str, Any]:
    entry = deb_metadata(path)
    if entry["package"] != args.expected_package:
        raise ValueError(f"{path}: package identity mismatch")
    if entry["version"] != args.product_version:
        raise ValueError(f"{path}: version mismatch")
    if entry["architecture"] not in {args.architecture, "all"}:
        raise ValueError(f"{path}: architecture mismatch")
    return entry


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def create_build(args: argparse.Namespace) -> None:
    validate_identity(args)
    debs = sorted(Path(args.deb_dir).glob("*.deb"))
    if len(debs) != 1:
        raise ValueError(f"expected exactly one Deb, found {len(debs)}")
    ci = {
        "run_id": str(args.ci_run_id),
        "workflow": args.ci_workflow,
        "workflow_ref": args.ci_workflow_ref,
    }
    if not all(ci.values()):
        raise ValueError("CI identity is incomplete")
    manifest = {
        "schema": BUILD_SCHEMA,
        "product": args.product,
        "version": args.product_version,
        "source_sha": args.source_sha,
        "distribution": args.distribution,
        "architecture": args.architecture,
        "ci": ci,
        "debs": [validated_deb(args, debs[0])],
    }
    destination = Path(args.output_dir) / (
        f"{args.product}_{args.distribution}_{args.architecture}.build.json"
    )
    write_json(destination, manifest)


def verify_build(args: argparse.Namespace) -> None:
    validate_identity(args)
    root = Path(args.artifact_dir).resolve(strict=True)
    matches: list[tuple[Path, Path]] = []
    for manifest_path in sorted(root.rglob("*.build.json")):
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if not isinstance(manifest, dict) or set(manifest) != BUILD_FIELDS:
            raise ValueError(f"build manifest fields are not exact: {manifest_path}")
        if manifest["schema"] != BUILD_SCHEMA:
            raise ValueError(f"unsupported build manifest schema: {manifest_path}")
        expected = {
            "product": args.product,
            "version": args.product_version,
            "source_sha": args.source_sha,
            "distribution": args.distribution,
            "architecture": args.architecture,
        }
        if any(manifest[key] != value for key, value in expected.items()):
            continue
        ci = manifest["ci"]
        if not isinstance(ci, dict) or set(ci) != CI_FIELDS or not all(ci.values()):
            raise ValueError(f"CI identity is invalid: {manifest_path}")
        if str(ci["run_id"]) != str(args.ci_run_id):
            continue
        entries = manifest["debs"]
        if not isinstance(entries, list) or len(entries) != 1:
            raise ValueError(f"manifest must contain exactly one Deb: {manifest_path}")
        declared = entries[0]
        if not isinstance(declared, dict) or set(declared) != DEB_FIELDS:
            raise ValueError(f"Deb manifest fields are not exact: {manifest_path}")
        filename = declared["file"]
        if not isinstance(filename, str) or Path(filename).name != filename:
            raise ValueError(f"unsafe Deb filename: {filename!r}")
        candidates = sorted(path for path in root.rglob(filename) if path.is_file())
        if len(candidates) != 1:
            raise ValueError(f"expected exactly one artifact named {filename}")
        actual = validated_deb(args, candidates[0])
        if actual != declared:
            raise ValueError(f"Deb metadata or digest mismatch: {filename}")
        matches.append((manifest_path, candidates[0]))
    if len(matches) != 1:
        raise ValueError(
            f"expected exactly one matching build manifest, found {len(matches)}"
        )

    manifest_path, deb = matches[0]
    deb_output = Path(args.deb_output_dir)
    manifest_output = Path(args.manifest_output_dir)
    deb_output.mkdir(parents=True, exist_ok=True)
    manifest_output.mkdir(parents=True, exist_ok=True)
    shutil.copy2(deb, deb_output / deb.name)
    shutil.copy2(manifest_path, manifest_output / manifest_path.name)


def add_identity_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--product", required=True)
    parser.add_argument("--expected-package", required=True)
    parser.add_argument("--product-version", required=True)
    parser.add_argument("--distribution", required=True)
    parser.add_argument("--architecture", required=True)
    parser.add_argument("--source-sha", required=True)
    parser.add_argument("--ci-run-id", required=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    build = subparsers.add_parser("build")
    add_identity_arguments(build)
    build.add_argument("--deb-dir", required=True)
    build.add_argument("--output-dir", required=True)
    build.add_argument("--ci-workflow", required=True)
    build.add_argument("--ci-workflow-ref", required=True)
    build.set_defaults(handler=create_build)

    verify = subparsers.add_parser("verify-build")
    add_identity_arguments(verify)
    verify.add_argument("--artifact-dir", required=True)
    verify.add_argument("--deb-output-dir", required=True)
    verify.add_argument("--manifest-output-dir", required=True)
    verify.set_defaults(handler=verify_build)
    args = parser.parse_args()
    try:
        args.handler(args)
    except (
        OSError,
        ValueError,
        subprocess.CalledProcessError,
        json.JSONDecodeError,
    ) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
