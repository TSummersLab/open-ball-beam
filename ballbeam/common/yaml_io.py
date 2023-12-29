"""YAML import/export utilities."""

import os
from typing import Any

from ruamel import yaml  # type: ignore[import]

from ballbeam.common.path_io import create_directory

Loader = yaml.Loader
Dumper = yaml.Dumper


def yaml_import(path: str):
    with open(path) as ymlfile:
        return yaml.load(ymlfile, Loader=Loader)


def yaml_export(dirname_out: str, filename_out: str, data: Any) -> None:
    create_directory(dirname_out)
    path = os.path.join(dirname_out, filename_out)
    with open(path, "w", encoding="utf-8") as yaml_file:
        dump = yaml.dump(data, default_flow_style=False, allow_unicode=True, encoding=None, Dumper=Dumper)
        yaml_file.write(dump)
        print(f"Wrote to {path}")
