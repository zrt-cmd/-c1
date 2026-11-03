#!/usr/bin/env python3
"""Apply dotted-key overrides to a YAML template and write a new params file."""
import argparse
import json

import yaml

parser = argparse.ArgumentParser()
parser.add_argument("--template", required=True)
parser.add_argument("--out", required=True)
parser.add_argument("--overrides", default="{}")
args = parser.parse_args()

with open(args.template, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)

for dotted, value in json.loads(args.overrides).items():
    keys = dotted.split(".")
    node = data
    for k in keys[:-1]:
        node = node[k]
    node[keys[-1]] = value

with open(args.out, "w", encoding="utf-8") as f:
    yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

print(args.out)
