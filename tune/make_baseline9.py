#!/usr/bin/env python3
"""baseline9: global costmap uses only the static map (no laser obstacle layer)."""
import yaml

src = "/home/zrtzrt/tune/nav2_baseline8.yaml"
dst = "/home/zrtzrt/tune/nav2_baseline9.yaml"
d = yaml.safe_load(open(src))
gc = d["global_costmap"]["global_costmap"]["ros__parameters"]
gc["plugins"] = ["static_layer", "inflation_layer"]
gc.pop("obstacle_layer", None)
yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst, "plugins:", gc["plugins"])
