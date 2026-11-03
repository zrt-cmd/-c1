#!/usr/bin/env python3
"""
RKLLM CLI chat client — interactive conversation with the RKLLM server.

Communicates via OpenAI-compatible HTTP API (/v1/chat/completions).
Supports streaming, non-streaming, and function calling (tool use) modes.

The RKLLM server must be running first:
  python3 llm_server.py

Usage:
  python3 llm_test.py                                      # interactive chat
  python3 llm_test.py --mode tools                         # function calling mode
  python3 llm_test.py --prompt "你好"                       # single-shot query
  python3 llm_test.py --server http://192.168.1.100:8080   # remote server
"""

import argparse
import json
import os
import readline
import sys
import textwrap

try:
    import requests
except ImportError:
    print("Please install requests: pip3 install requests")
    sys.exit(1)

from tools_config import TOOLS, FUNCTION_MAP, parse_tool_calls, execute_tool_calls


class LLMClient:
    """Minimal OpenAI-compatible client for RKLLM Server."""

    def __init__(self, base_url="http://127.0.0.1:8080", timeout=120):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.session = requests.Session()
        self.session.keep_alive = False
        adapter = requests.adapters.HTTPAdapter(max_retries=3)
        self.session.mount("http://", adapter)

    def list_models(self):
        r = self.session.get(f"{self.base_url}/v1/models", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def _build_request(self, messages, temperature, top_p, top_k, max_tokens,
                        repeat_penalty, enable_thinking, tools):
        data = {
            "model": "rkllm",
            "messages": messages,
            "temperature": temperature,
            "top_p": top_p,
            "top_k": top_k,
            "max_tokens": max_tokens,
            "repeat_penalty": repeat_penalty,
            "frequency_penalty": 0.0,
            "presence_penalty": 0.0,
            "enable_thinking": enable_thinking,
        }
        if tools is not None:
            data["tools"] = tools
        return data

    def chat(self, messages, temperature=0.8, top_p=0.9, top_k=1,
             max_tokens=4096, repeat_penalty=1.1, enable_thinking=False,
             tools=None):
        """Streaming chat — yields content chunks as they arrive.

        Do NOT add 'return' to this function; any 'return' would be
        swallowed by the generator protocol.  Use chat_once() for
        non-streaming calls.
        """
        data = self._build_request(messages, temperature, top_p, top_k,
                                   max_tokens, repeat_penalty,
                                   enable_thinking, tools)
        data["stream"] = True

        headers = {"Content-Type": "application/json", "Authorization": "not_required"}
        resp = self.session.post(
            f"{self.base_url}/v1/chat/completions",
            json=data, headers=headers, stream=True, timeout=self.timeout,
        )
        if resp.status_code != 200:
            raise RuntimeError(f"Server error ({resp.status_code}): {resp.text}")

        for line in resp.iter_lines(decode_unicode=True):
            if line.startswith("data: ") and line[6:].strip() != "[DONE]":
                chunk = json.loads(line[6:])
                delta = chunk["choices"][0].get("delta", {})
                content = delta.get("content", "")
                if content:
                    yield content

    def chat_once(self, messages, temperature=0.8, top_p=0.9, top_k=1,
                  max_tokens=4096, repeat_penalty=1.1, enable_thinking=False,
                  tools=None):
        """Non-streaming chat — returns the full response dict.

        Separate method (no 'yield') so Python returns a real dict, not a
        generator.  Required for function calling where we need to inspect
        the message content for <tool_call> blocks.
        """
        data = self._build_request(messages, temperature, top_p, top_k,
                                   max_tokens, repeat_penalty,
                                   enable_thinking, tools)
        data["stream"] = False

        headers = {"Content-Type": "application/json", "Authorization": "not_required"}
        resp = self.session.post(
            f"{self.base_url}/v1/chat/completions",
            json=data, headers=headers, stream=False, timeout=self.timeout,
        )
        if resp.status_code != 200:
            raise RuntimeError(f"Server error ({resp.status_code}): {resp.text}")
        return resp.json()


# ---------------------------------------------------------------------------
# Function calling loop
# ---------------------------------------------------------------------------

def run_with_tools(client, messages, args):
    """Single-shot function calling — returns raw tool result JSON.

    First call uses non-streaming (chat_once) to minimise overhead since
    we buffer silently anyway.  No second model call — just execute the
    tool and return its JSON result directly.
    """
    print("AI  > ", end="", flush=True)

    # --- First call: non-streaming, check for tool_call ---
    resp = client.chat_once(
        messages, temperature=args.temperature,
        top_p=args.top_p, max_tokens=args.max_tokens,
        repeat_penalty=args.repeat_penalty,
        enable_thinking=args.enable_thinking,
        tools=TOOLS,
    )
    full = resp["choices"][0]["message"]["content"]

    tool_calls = parse_tool_calls(full)
    if not tool_calls:
        # Model didn't need tools — print directly
        print(full)
        messages.append({"role": "assistant", "content": full})
        return full

    # Execute tools, return raw JSON — no second model round
    names = [tc.get("name", "?") for tc in tool_calls]
    _, tool_msgs = execute_tool_calls(tool_calls)
    messages.extend(tool_msgs)

    for tmsg in tool_msgs:
        result_obj = json.loads(tmsg["content"])
        print(json.dumps(result_obj, ensure_ascii=False, indent=2))

    return json.loads(tool_msgs[0]["content"])


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def print_banner(server_url, model_name, mode="chat"):
    label = {
        "chat": "RKLLM Chat",
        "tools": "RKLLM Chat (Function Calling)",
    }.get(mode, "RKLLM Chat")

    print()
    print("=" * 60)
    print(f"  {label}  |  {model_name}")
    print(f"  Server: {server_url}")
    print("=" * 60)
    print()


def main():
    parser = argparse.ArgumentParser(
        description="RKLLM CLI chat client",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 llm_test.py                                          # interactive chat
  python3 llm_test.py --mode tools                             # function calling mode
  python3 llm_test.py --prompt "你好"                           # single shot
  python3 llm_test.py --server http://192.168.1.100:8080       # remote server
  python3 llm_test.py --temperature 0.5 --max-tokens 512        # tuning
        """,
    )

    parser.add_argument("--server", default="http://127.0.0.1:8080",
                        help="RKLLM server URL (default: http://127.0.0.1:8080)")
    parser.add_argument("--mode", default="chat", choices=["chat", "tools"],
                        help="chat = normal conversation, tools = function calling (default: chat)")
    parser.add_argument("--prompt", default=None,
                        help="Single-shot prompt (non-interactive mode)")
    parser.add_argument("--no-stream", action="store_true",
                        help="Disable streaming (wait for full response)")
    parser.add_argument("--temperature", type=float, default=0.8)
    parser.add_argument("--top-p", type=float, default=0.9)
    parser.add_argument("--max-tokens", type=int, default=4096)
    parser.add_argument("--repeat-penalty", type=float, default=1.1)
    parser.add_argument("--enable-thinking", action="store_true")
    parser.add_argument("--timeout", type=int, default=120)

    args = parser.parse_args()

    # In tools mode, use shorter max_tokens (tool calls are brief) and lower
    # temperature (more deterministic), unless the user explicitly overrides.
    if args.mode == "tools":
        if "--max-tokens" not in sys.argv:
            args.max_tokens = 128
        if "--temperature" not in sys.argv:
            args.temperature = 0.3

    stream = not args.no_stream

    client = LLMClient(base_url=args.server, timeout=args.timeout)

    # Verify server connectivity
    try:
        models = client.list_models()
        model_name = models["data"][0]["id"] if models.get("data") else "rkllm"
    except Exception as e:
        print(f"Error: Cannot connect to server at {args.server}")
        print(f"  {e}")
        print()
        print("Make sure the RKLLM server is running:")
        print("  python3 llm_server.py")
        sys.exit(1)

    # -------------------------------------------------------------------
    # Single-shot mode
    # -------------------------------------------------------------------
    if args.prompt:
        messages = [{"role": "user", "content": args.prompt}]

        if args.mode == "tools":
            run_with_tools(client, messages, args)
            return

        if stream:
            for chunk in client.chat(
                messages, temperature=args.temperature,
                top_p=args.top_p, max_tokens=args.max_tokens,
                repeat_penalty=args.repeat_penalty,
                enable_thinking=args.enable_thinking,
            ):
                print(chunk, end="", flush=True)
            print()
        else:
            resp = client.chat_once(
                messages, temperature=args.temperature,
                top_p=args.top_p, max_tokens=args.max_tokens,
                repeat_penalty=args.repeat_penalty,
                enable_thinking=args.enable_thinking,
            )
            print(resp["choices"][0]["message"]["content"])
        return

    # -------------------------------------------------------------------
    # Interactive mode
    # -------------------------------------------------------------------
    print_banner(args.server, model_name, args.mode)

    if args.mode == "tools":
        print("  Function calling 模式 — 模型可以调用工具函数")
        print("  可用工具: " + ", ".join(t["function"]["name"] for t in TOOLS))
        print("  输入问题后回车，Ctrl+C 退出")
        print()

    messages = []

    try:
        while True:
            try:
                user_input = input("You > ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not user_input:
                continue
            if user_input.lower() in ("exit", "quit", "q"):
                break
            if user_input.lower() == "clear":
                messages.clear()
                print("  [history cleared]")
                continue
            if user_input.lower() == "tools":
                print("  Available tools:")
                for t in TOOLS:
                    print(f"    - {t['function']['name']}: {t['function']['description']}")
                continue

            messages.append({"role": "user", "content": user_input})

            if args.mode == "tools":
                # Function calling mode — model may invoke tools
                try:
                    run_with_tools(client, messages, args)
                except Exception as e:
                    print(f"\n  [Error] {e}")
                    if messages and messages[-1]["role"] == "user":
                        messages.pop()
                continue

            # Normal chat mode — streaming
            print("AI  > ", end="", flush=True)

            try:
                if stream:
                    full_response = ""
                    for chunk in client.chat(
                        messages, temperature=args.temperature,
                        top_p=args.top_p, max_tokens=args.max_tokens,
                        repeat_penalty=args.repeat_penalty,
                        enable_thinking=args.enable_thinking,
                    ):
                        print(chunk, end="", flush=True)
                        full_response += chunk
                    print()
                else:
                    print("...", end="", flush=True)
                    resp = client.chat_once(
                        messages, temperature=args.temperature,
                        top_p=args.top_p, max_tokens=args.max_tokens,
                        repeat_penalty=args.repeat_penalty,
                        enable_thinking=args.enable_thinking,
                    )
                    full_response = resp["choices"][0]["message"]["content"]
                    print(f"\rAI  > {full_response}")

                messages.append({"role": "assistant", "content": full_response})

            except Exception as e:
                print(f"\n  [Error] {e}")
                if messages and messages[-1]["role"] == "user":
                    messages.pop()

    except KeyboardInterrupt:
        print()

    print("Goodbye.")


if __name__ == "__main__":
    main()
