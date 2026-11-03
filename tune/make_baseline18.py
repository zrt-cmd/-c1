#!/usr/bin/env python3
"""baseline18: local costmap treats unknown space as free (RPP collision check fix)."""
import yaml

src = "/home/zrtzrt/tune/nav2_baseline17.yaml"
dst = "/home/zrtzrt/tune/nav2_baseline18.yaml"
d = yaml.safe_load(open(src))

lc = d["local_costmap"]["local_costmap"]["ros__parameters"]
lc["track_unknown_space"] = False

yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst, "local track_unknown_space =", lc["track_unknown_space"])
