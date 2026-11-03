#!/usr/bin/env python3
"""baseline10: centered global path (0.09), forgiving local costmap (0.04)."""
import yaml

src = "/home/zrtzrt/tune/nav2_baseline9.yaml"
dst = "/home/zrtzrt/tune/nav2_baseline10.yaml"
d = yaml.safe_load(open(src))
d["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"] = 0.09
d["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"] = 0.04
yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst)
