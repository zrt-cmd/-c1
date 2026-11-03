"""实验 06 共享测试夹具：src 导入路径。"""
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))
