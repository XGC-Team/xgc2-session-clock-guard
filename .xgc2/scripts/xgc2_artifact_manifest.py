#!/usr/bin/env python3
"""Create strict XGC2 trusted build manifests for this product."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path


BUILD_SCHEMA = "xgc2.build-artifact.v1"
HEX40_OR_64 = re.compile(r"^[0-9a-f]{40}(?:[0-9a-f]{24})?$")
TOKEN = re.compile(r"^[a-z0-9][a-z0-9.+_-]*$")


def deb_field(path: Path, name: str) -> str:
    return subprocess.check_output(
        ["dpkg-deb", "-f", str(path), name], text=True
    ).strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_token(value: str, name: str) -> str:
    if not TOKEN.fullmatch(value):
        raise ValueError(f"{name} is not canonical: {value!r}")
    return value


def create_build(args: argparse.Namespace) -> None:
    if not HEX40_OR_64.fullmatch(args.source_sha):
        raise ValueError("source SHA must be 40 or 64 lowercase hex characters")
    product = canonical_token(args.product, "product")
    expected_package = canonical_token(args.expected_package, "package")
    distribution = canonical_token(args.distribution, "distribution")
    architecture = canonical_token(args.architecture, "architecture")
    debs = sorted(Path(args.deb_dir).rglob("*.deb"))
    if len(debs) != 1:
        raise ValueError(f"expected exactly one deb, found {len(debs)}")
    deb = debs[0]
    package = deb_field(deb, "Package")
    version = deb_field(deb, "Version")
    deb_architecture = deb_field(deb, "Architecture")
    if package != expected_package:
        raise ValueError(f"artifact package {package} != {expected_package}")
    if version != args.product_version:
        raise ValueError(f"artifact version {version} != {args.product_version}")
    if deb_architecture not in (architecture, "all"):
        raise ValueError(
            f"artifact architecture {deb_architecture} is incompatible with {architecture}"
        )

    payload = {
        "schema": BUILD_SCHEMA,
        "product": product,
        "source_sha": args.source_sha,
        "version": args.product_version,
        "distribution": distribution,
        "architecture": architecture,
        "ci": {
            "run_id": str(args.ci_run_id),
            "workflow": args.ci_workflow,
            "workflow_ref": args.ci_workflow_ref,
        },
        "created_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "debs": [
            {
                "file": deb.name,
                "package": package,
                "version": version,
                "architecture": deb_architecture,
                "sha256": sha256(deb),
                "size": deb.stat().st_size,
            }
        ],
    }
    output = Path(args.output_dir) / (
        f"{product}_{distribution}_{architecture}.build.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    subcommands = parser.add_subparsers(dest="command", required=True)
    build = subcommands.add_parser("build")
    build.add_argument("--deb-dir", required=True)
    build.add_argument("--output-dir", required=True)
    build.add_argument("--product", required=True)
    build.add_argument("--expected-package", required=True)
    build.add_argument("--product-version", required=True)
    build.add_argument("--distribution", required=True)
    build.add_argument("--architecture", required=True)
    build.add_argument("--source-sha", required=True)
    build.add_argument("--ci-run-id", required=True)
    build.add_argument("--ci-workflow", required=True)
    build.add_argument("--ci-workflow-ref", required=True)
    args = parser.parse_args()
    if args.command == "build":
        create_build(args)


if __name__ == "__main__":
    main()
