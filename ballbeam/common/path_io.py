"""Path input and output utilities."""
from __future__ import annotations

from pathlib import Path


def create_directory(path: Path | str) -> None:
    """Create target directory & all intermediate directories if nonexistent."""
    path = Path(path)
    if not path.exists():
        print(f"Directory created at {path}")
        path.mkdir(parents=True, exist_ok=True)
