#!/usr/bin/env python
"""Write a copy of PackageContents.xml with AppVersion set to a requested version."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Path to source PackageContents.xml")
    parser.add_argument("--output", required=True, help="Path to generated PackageContents.xml")
    parser.add_argument("--version", required=True, help="Version string to write to AppVersion")
    args = parser.parse_args()

    source = Path(args.input)
    target = Path(args.output)

    xml = source.read_text(encoding="utf-8")
    updated, count = re.subn(r'AppVersion="[^"]+"', f'AppVersion="{args.version}"', xml, count=1)
    if count != 1:
        raise RuntimeError("Could not find exactly one AppVersion attribute in PackageContents.xml")

    target.write_text(updated, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
