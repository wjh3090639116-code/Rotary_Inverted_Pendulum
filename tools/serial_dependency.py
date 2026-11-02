"""为命令行辅助工具提供工程内的串口依赖。"""
from pathlib import Path
import sys

dependency = Path(__file__).resolve().parents[1] / "dependencies" / "pyserial.zip"
if dependency.is_file():
    sys.path.insert(0, str(dependency))
