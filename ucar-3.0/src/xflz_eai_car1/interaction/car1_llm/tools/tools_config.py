#!/usr/bin/env python3
"""
Tools for RKLLM function calling demo.

Add new tools by adding entries to both TOOLS and FUNCTION_MAP.
Keep descriptions minimal — each extra character adds to prefill delay.
"""

import datetime
import json
import re
import subprocess


# ---------------------------------------------------------------------------
# Tool implementations
# ---------------------------------------------------------------------------

def get_current_time():
    now = datetime.datetime.now()
    return {
        "datetime": now.strftime("%Y-%m-%d %H:%M:%S"),
        "date": now.strftime("%Y-%m-%d"),
        "time": now.strftime("%H:%M:%S"),
        "weekday": now.strftime("%A"),
    }


def get_system_info():
    try:
        uptime = subprocess.check_output(["uptime", "-p"], text=True).strip()
    except Exception:
        uptime = "unknown"
    try:
        temp_raw = subprocess.check_output(
            ["cat", "/sys/class/thermal/thermal_zone0/temp"], text=True
        ).strip()
        cpu_temp = int(temp_raw) / 1000.0
    except Exception:
        cpu_temp = 0.0
    return {
        "hostname": subprocess.check_output(["hostname"], text=True).strip(),
        "uptime": uptime,
        "cpu_temperature_celsius": cpu_temp,
    }


def calculator(expression: str):
    if not re.match(r'^[\d+\-*/().%\s]+$', expression):
        return {"error": "unsafe"}
    try:
        return {"result": eval(expression, {"__builtins__": {}}, {})}
    except Exception as e:
        return {"error": str(e)}


def get_battery_status():
    return {"voltage_mv": 11800, "percentage": 85}


FUNCTION_MAP = {
    "get_current_time": get_current_time,
    "get_system_info": get_system_info,
    "calculator": calculator,
    "get_battery_status": get_battery_status,
}


# ---------------------------------------------------------------------------
# Tools schema — stripped to minimum
# ---------------------------------------------------------------------------

TOOLS = [
    {"type": "function", "function": {"name": "get_current_time", "description": "当前时间"}},
    {"type": "function", "function": {"name": "get_system_info", "description": "系统信息"}},
    {"type": "function", "function": {"name": "calculator", "description": "计算"}},
    {"type": "function", "function": {"name": "get_battery_status", "description": "电量"}},
]


# ---------------------------------------------------------------------------
# Tool call parser
# ---------------------------------------------------------------------------

def parse_tool_calls(text: str):
    json_pattern = r"<tool_call>\s*(\{.*?\})\s*</tool_call>"
    json_matches = re.findall(json_pattern, text, re.DOTALL)
    if json_matches:
        return [json.loads(m) for m in json_matches]

    results = []
    blocks = re.findall(r"<tool_call>\s*(.*?)\s*</tool_call>", text, re.DOTALL)
    for block in blocks:
        func_blocks = re.findall(
            r"<function=(\S+)>\s*(.*?)\s*</function>", block, re.DOTALL
        )
        for func_name, params_block in func_blocks:
            arguments = {}
            for param_name, param_value in re.findall(
                r"<parameter=(\S+)>\s*(.*?)\s*</parameter>", params_block, re.DOTALL
            ):
                arguments[param_name] = param_value.strip()
            results.append({"name": func_name, "arguments": arguments})
    return results


def execute_tool_calls(tool_calls):
    normalized = []
    tool_msgs = []
    for tc in tool_calls:
        fn = tc.get("function", tc)
        name = fn.get("name", "")
        args = fn.get("arguments", {})
        result = FUNCTION_MAP[name](**args) if name in FUNCTION_MAP else {"error": f"Unknown: {name}"}
        normalized.append({
            "id": f"call_{name}",
            "type": "function",
            "function": {"name": name, "arguments": json.dumps(args, ensure_ascii=False)},
        })
        tool_msgs.append({"role": "tool", "name": name, "content": json.dumps(result, ensure_ascii=False)})
    assistant_msg = {"role": "assistant", "content": None, "tool_calls": normalized}
    return assistant_msg, tool_msgs
