#!/usr/bin/env python3
"""baseline11: lifecycle managers use wall time (immune to sim clock jumps)."""
import yaml

src = "/home/zrtzrt/tune/nav2_baseline10.yaml"
dst = "/home/zrtzrt/tune/nav2_baseline11.yaml"
d = yaml.safe_load(open(src))
for key in ("lifecycle_manager_localization", "lifecycle_manager_navigation"):
    d[key]["ros__parameters"]["use_sim_time"] = False
yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst)
