from ruamel import yaml

Loader = yaml.Loader
Dumper = yaml.Dumper


def yaml_import(path):
    with open(path, "r") as ymlfile:
        return yaml.load(ymlfile, Loader=Loader)


def yaml_export(data, path):
    with open(path, 'w', encoding="utf-8") as yaml_file:
        dump = yaml.dump(data, default_flow_style=False, allow_unicode=True, encoding=None, Dumper=Dumper)
        yaml_file.write(dump)
    return
