#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def field(path: Path, name: str) -> str:
    return subprocess.check_output(
        ["dpkg-deb", "-f", str(path), name], text=True
    ).strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def product_version(root: Path) -> str:
    for line in (root / ".xgc2/product.yml").read_text(encoding="utf-8").splitlines():
        if line.startswith("version:"):
            return line.split(":", 1)[1].strip()
    raise ValueError("product version is missing")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--deb-dir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--architecture", required=True, choices=("amd64", "arm64"))
    parser.add_argument("--source-sha", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--workflow-ref", required=True)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[2]
    debs = sorted(Path(args.deb_dir).glob("*.deb"))
    if len(debs) != 1:
        raise ValueError(f"expected exactly one deb, found {len(debs)}")
    deb = debs[0]
    actual_architecture = field(deb, "Architecture")
    if actual_architecture != args.architecture:
        raise ValueError(
            f"artifact architecture {actual_architecture} != matrix {args.architecture}"
        )
    expected_version = product_version(root)
    actual_version = field(deb, "Version")
    if actual_version != expected_version:
        raise ValueError(f"artifact version {actual_version} != {expected_version}")

    payload = {
        "schema": "xgc2.build-artifact.v1",
        "product": "xgc2-session-clock-guard",
        "version": expected_version,
        "distribution": "focal",
        "ros_distro": "noetic",
        "architecture": args.architecture,
        "source_sha": args.source_sha,
        "ci": {"run_id": args.run_id, "workflow_ref": args.workflow_ref},
        "created_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "debs": [
            {
                "file": deb.name,
                "package": field(deb, "Package"),
                "version": actual_version,
                "architecture": actual_architecture,
                "sha256": sha256(deb),
                "size": deb.stat().st_size,
            }
        ],
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
