#!/usr/bin/env python3
"""baseline13: global inflation = footprint inscribed radius (whole body clears walls)."""
import yaml

src = "/home/zrtzrt/tune/nav2_baseline11.yaml"
dst = "/home/zrtzrt/tune/nav2_baseline13.yaml"
d = yaml.safe_load(open(src))
d["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"] = 0.145
yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst)
