#!/usr/bin/env python3
"""Make a params file from the template with inflation overrides.

Usage: mkparams.py SRC DST GLOBAL_INFL [LOCAL_INFL]
"""
import sys

import yaml

src, dst, g = sys.argv[1], sys.argv[2], float(sys.argv[3])
loc = float(sys.argv[4]) if len(sys.argv) > 4 else g
d = yaml.safe_load(open(src))
d["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"] = g
d["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"] = loc
yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst, "global", g, "local", loc)
