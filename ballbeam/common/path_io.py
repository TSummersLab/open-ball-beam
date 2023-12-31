"""Path input and output utilities."""

from pathlib import Path


def create_directory(dirname_out: str) -> None:
    """Create target directory & all intermediate directories if nonexistent."""
    path = Path(dirname_out)
    if not path.exists():
        print(f"Directory created at {dirname_out}")
    path.mkdir(parents=True, exist_ok=True)
