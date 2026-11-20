"""每个示例都要能从仓库根直接跑起来：不装包、不设 PYTHONPATH。

示例脚本都在文件头把仓库的 ``src/`` 插进 ``sys.path``，所以 ``--help`` 不需要
安装、也不需要硬件。``-S`` 关掉 site-packages，``PYTHONPATH`` 也清掉，于是
``import litegrip`` 只可能来自脚本自己插进去的那个 ``src/`` —— 这正是要断言的
事。任何一个示例漏掉引导，或者顶层 import 了需要硬件的模块，这里都会红。
"""

from __future__ import annotations

import glob
import os
import subprocess
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestEveryExampleStarts(unittest.TestCase):
    def test_each_example_answers_help_on_a_bare_interpreter(self):
        scripts = sorted(glob.glob(os.path.join(ROOT, "examples", "*.py")))
        self.assertTrue(scripts, "examples/ 下一个脚本都没有")

        env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
        for script in scripts:
            # 照 README 的写法：工作目录是仓库根，命令行里带 examples/。
            relative = os.path.relpath(script, ROOT)
            with self.subTest(script=relative):
                proc = subprocess.run(
                    [sys.executable, "-S", relative, "--help"],
                    cwd=ROOT, env=env,
                    capture_output=True, text=True, timeout=60)
                self.assertEqual(
                    proc.returncode, 0,
                    f"{relative} --help 退出 {proc.returncode}：\n{proc.stderr}")


if __name__ == "__main__":
    unittest.main()
