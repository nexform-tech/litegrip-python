"""每个示例都要能从仓库根直接跑起来：不装包、不设 PYTHONPATH（#47）。

``trajectory.py`` 曾经是六个脚本里唯一没有 ``sys.path`` 引导的那个，而 README 把它
列进「都能从仓库根跑」那张表，还把 ``--list`` 当作不需要硬件的第一步来推荐，于是
照文档敲下去得到 ``ModuleNotFoundError``。

``-S`` 关掉 site-packages，``PYTHONPATH`` 也清掉，所以 ``import litegrip`` 只可能来自
脚本自己插进去的那个 ``src/`` —— 这正是要断言的事。
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
