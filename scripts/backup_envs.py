#!/usr/bin/env python3
"""Back up every tracked .env file in the repo into a single timestamped archive."""

import argparse
import sys
import tarfile
import zipfile
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
BACKUP_DIR = REPO_ROOT / "env_backups"
SKIP_DIRS = {".git", "node_modules", "env_backups"}


def find_env_files() -> list[Path]:
    found = []
    for path in REPO_ROOT.rglob(".env*"):
        if not path.is_file() or path.name.endswith(".example"):
            continue
        if any(part in SKIP_DIRS for part in path.relative_to(REPO_ROOT).parts):
            continue
        found.append(path)
    return sorted(found)


def make_archive(fmt: str) -> Path:
    env_files = find_env_files()
    if not env_files:
        print("No .env files found.", file=sys.stderr)
        sys.exit(1)

    BACKUP_DIR.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    archive_path = BACKUP_DIR / f"env_backup_{timestamp}.{fmt}"

    if fmt == "zip":
        with zipfile.ZipFile(archive_path, "w", zipfile.ZIP_DEFLATED) as zf:
            for f in env_files:
                zf.write(f, f.relative_to(REPO_ROOT))
    else:
        with tarfile.open(archive_path, "w:gz") as tf:
            for f in env_files:
                tf.add(f, arcname=f.relative_to(REPO_ROOT))

    print(f"Backed up {len(env_files)} .env file(s) to {archive_path}")
    for f in env_files:
        print(f"  - {f.relative_to(REPO_ROOT)}")
    return archive_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--format", choices=["zip", "tar"], default="zip", help="Archive format (default: zip)")
    args = parser.parse_args()
    make_archive(args.format)


if __name__ == "__main__":
    main()
