"""YAML import/export utilities."""
from __future__ import annotations

from pathlib import Path
from typing import Any

from ruamel import yaml  # type: ignore[import]

from ballbeam.common.path_io import create_directory

Loader = yaml.Loader
Dumper = yaml.Dumper


def yaml_import(path: str | Path) -> Any:
    """Import data from a YAML file."""
    if isinstance(path, str):
        path = Path(path)
    with path.open() as ymlfile:
        return yaml.load(ymlfile, Loader=Loader)


def yaml_export(dirname_out: str, filename_out: str, data: Any) -> None:
    """Export data to a YAML file."""
    create_directory(dirname_out)
    path = Path(dirname_out) / filename_out
    with path.open("w", encoding="utf-8") as yaml_file:
        dump = yaml.dump(data, default_flow_style=False, allow_unicode=True, encoding=None, Dumper=Dumper)
        yaml_file.write(dump)
        print(f"Wrote to {path}")
