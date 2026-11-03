#!/usr/bin/env python3
"""baseline20: lower global inflation so replanning from near-wall poses works."""
import yaml

src = "/home/zrtzrt/tune/nav2_baseline19.yaml"
dst = "/home/zrtzrt/tune/nav2_baseline20.yaml"
d = yaml.safe_load(open(src))
d["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"] = 0.05
yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst)
