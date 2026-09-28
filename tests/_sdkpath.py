"""把仓库的 ``src/`` 插进 sys.path —— 测试不打包安装也能 import litegrip。"""

import os
import sys

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
