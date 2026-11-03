#!/usr/bin/env python3
"""baseline16: local costmap inflation 0.02 and no footprint padding."""
import yaml

src = "/home/zrtzrt/tune/nav2_baseline15.yaml"
dst = "/home/zrtzrt/tune/nav2_baseline16.yaml"
d = yaml.safe_load(open(src))

lc = d["local_costmap"]["local_costmap"]["ros__parameters"]
lc["inflation_layer"]["inflation_radius"] = 0.02
lc["footprint_padding"] = 0.0
gc = d["global_costmap"]["global_costmap"]["ros__parameters"]
gc["footprint_padding"] = 0.0

yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst)
print("local inflation 0.02, padding 0")
