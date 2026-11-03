#!/usr/bin/env python3
"""Optuna study for the fixed math_room G->C task.

Collision is a hard failure; time is only a soft penalty.
Run with the tuning venv:
  ~/tune/venv/bin/python ~/tune/study.py --trials 20
"""
import argparse
import json
import os
import subprocess

import optuna
import yaml

HOME = os.path.expanduser("~")
TEMPLATE = os.path.join(HOME, "tune/nav2_sim_template.yaml")
RUNNER = os.path.join(HOME, "tune/run_episode.py")
RUNS = os.path.join(HOME, "tune/runs")
os.makedirs(RUNS, exist_ok=True)

CS = "controller_server/ros__parameters"
FP = CS + "/FollowPath"
LC = "local_costmap/local_costmap/ros__parameters/inflation_layer"
GC = "global_costmap/global_costmap/ros__parameters/inflation_layer"


def sample_params(trial):
    return {
        LC + "/inflation_radius": trial.suggest_float("local_inflation", 0.05, 0.09),
        GC + "/inflation_radius": trial.suggest_float("global_inflation", 0.05, 0.09),
        FP + "/max_vel_x": trial.suggest_float("max_vel_x", 0.03, 0.12),
        FP + "/acc_lim_x": trial.suggest_float("acc_lim_x", 0.2, 1.0),
        FP + "/max_vel_theta": trial.suggest_float("max_vel_theta", 0.5, 1.5),
        FP + "/sim_time": trial.suggest_float("sim_time", 1.0, 3.0),
        FP + "/vx_samples": trial.suggest_int("vx_samples", 10, 40),
        FP + "/vtheta_samples": trial.suggest_int("vtheta_samples", 20, 60),
        FP + "/min_vel_theta": trial.suggest_float("min_vel_theta", 0.1, 0.3),
        FP + "/PathDist.scale": trial.suggest_float("path_dist_scale", 20.0, 80.0),
        FP + "/BaseObstacle.scale": trial.suggest_float("base_obstacle_scale", 0.005, 0.1, log=True),
        FP + "/PathAlign.scale": trial.suggest_float("path_align_scale", 20.0, 80.0),
        CS + "/controller_frequency": trial.suggest_float("controller_frequency", 10.0, 20.0),
    }


def write_params(overrides, path):
    with open(TEMPLATE, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    for dotted, value in overrides.items():
        keys = dotted.split("/")
        node = data
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def objective_factory(args):
    def objective(trial):
        overrides = sample_params(trial)
        n = trial.number
        params_path = os.path.join(RUNS, f"trial_{n:04d}.yaml")
        out_path = os.path.join(RUNS, f"trial_{n:04d}.json")
        write_params(overrides, params_path)
        domain = 100 + (trial.number % 50)
        partition = f"gp{trial.number}"

        cmd = [
            "bash", "-lc",
            "source /opt/ros/jazzy/setup.bash && "
            "source ~/ros2_ws/install/setup.bash && "
            f"python3 {RUNNER} --params {params_path} --out {out_path} "
            f"--timeout {args.episode_timeout} --domain {domain} --partition {partition}",
        ]
        try:
            subprocess.run(
                cmd, timeout=args.episode_timeout + 150, check=False,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        except subprocess.TimeoutExpired:
            trial.set_user_attr("metrics", {"status": "hard_timeout"})
            return -2000.0

        try:
            with open(out_path, "r", encoding="utf-8") as f:
                m = json.load(f)
        except Exception:
            trial.set_user_attr("metrics", {"status": "no_metrics"})
            return -2000.0

        trial.set_user_attr("metrics", m)
        clr = m.get("min_clearance")
        if m.get("collided"):
            # hard constraint: never reward a collision
            return -1000.0 - 1000.0 * max(0.0, 0.08 - (clr or 0.0))
        if not m.get("success"):
            fd = m.get("final_dist")
            return -500.0 - (0.0 if fd is None else 20.0 * fd)
        t = m.get("time") or 999.0
        return 1000.0 - 0.5 * t + 50.0 * (clr or 0.0)

    return objective


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--episode-timeout", type=float, default=150.0)
    ap.add_argument("--study-name", default="g2c_stage1")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n-startup-trials", type=int, default=8)
    ap.add_argument("--stop-on-success", type=int, default=0)
    args = ap.parse_args()

    storage = f"sqlite:///{os.path.join(HOME, 'tune/study.db')}"
    sampler = optuna.samplers.TPESampler(
        seed=args.seed, n_startup_trials=args.n_startup_trials
    )
    study = optuna.create_study(
        study_name=args.study_name,
        storage=storage,
        load_if_exists=True,
        direction="maximize",
        sampler=sampler,
    )
    successes = 0

    def callback(study, trial):
        nonlocal successes
        m = trial.user_attrs.get("metrics", {})
        if m.get("success"):
            successes += 1
            print(f"SUCCESS_COUNT {successes} at trial {trial.number}", flush=True)
        if args.stop_on_success and successes >= args.stop_on_success:
            print(f"STOP: reached {successes} successful trials", flush=True)
            study.stop()

    study.optimize(
        objective_factory(args), n_trials=args.trials, callbacks=[callback]
    )

    print("BEST_VALUE", study.best_value)
    print("BEST_PARAMS", json.dumps(study.best_params, indent=2))
    print("BEST_METRICS", json.dumps(study.best_trial.user_attrs.get("metrics"), indent=2))


if __name__ == "__main__":
    main()
