#!/usr/bin/env python3
"""Repair the malformed margin in the project's pinned Perception Inspector UI."""

import argparse
from pathlib import Path


BAD_MARGIN = b"margin: 1px, 1px, 1px, 3px;"
GOOD_MARGIN = b"margin: 1px 1px 1px 3px;"
UI_FILE = Path(
    "Library/PackageCache/com.unity.perception@0.8.0-preview.3/"
    "Editor/Randomization/Uxml/Sampler/FloatRangeElement.uxml"
)


def repair(unity_project):
    path = unity_project / UI_FILE
    if not path.is_file():
        raise ValueError(
            "Perception 0.8.0-preview.3's cached UI file was not found. "
            "Open this Unity project once to download its packages, then retry: "
            + str(path)
        )
    original = path.read_bytes()
    if original.count(BAD_MARGIN) == 1:
        # Preserve the asset's BOM, line endings, and all other styles and bindings.
        path.write_bytes(original.replace(BAD_MARGIN, GOOD_MARGIN, 1))
        print("Corrected Perception FloatRange Inspector margin: " + str(path))
    elif BAD_MARGIN not in original and GOOD_MARGIN in original:
        print("Perception FloatRange Inspector margin is already corrected.")
    else:
        raise ValueError("Unexpected UI file contents; left unchanged: " + str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "unity_project", nargs="?", type=Path,
        default=Path(__file__).resolve().parents[1] / "UnityProject",
        help="UnityProject folder (defaults to this repository's UnityProject)",
    )
    args = parser.parse_args()
    try:
        repair(args.unity_project.resolve())
    except (OSError, ValueError) as error:
        parser.exit(1, str(error) + "\n")


if __name__ == "__main__":
    main()
