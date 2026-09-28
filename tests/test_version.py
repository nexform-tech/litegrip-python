"""``litegrip.__version__`` 的来源：装过的发行版元数据优先，裸源码树打标记。

硬编码一个版本号在这里没有意义 —— 仓库从不把版本回落（AGENTS.md §3），
semantic-release 只在发布工作区里改写清单，所以 git tag 是唯一真相。写死的
数字只会是一份永远过期的副本。
"""

import _sdkpath  # noqa: F401  (把 src/ 插进 sys.path)
import importlib.metadata as _md
import unittest

import litegrip


class TestVersionResolution(unittest.TestCase):
    """`_detect_version` 的两条分支 + 导入时刻的实际取值。"""

    def setUp(self):
        self._orig = _md.version
        self.addCleanup(setattr, _md, "version", self._orig)

    def test_prefers_the_installed_distribution(self):
        _md.version = lambda name: "9.9.9"
        self.assertEqual(litegrip._detect_version(), "9.9.9")

    def test_marks_a_source_tree_checkout(self):
        """查不到发行版元数据时给出可辨认的标记，而不是某个假版本号。"""

        def missing(name):
            raise _md.PackageNotFoundError(name)

        _md.version = missing
        self.assertEqual(litegrip._detect_version(), litegrip._VERSION_SOURCE_TREE)
        self.assertEqual(litegrip._VERSION_SOURCE_TREE, "0.0.0+source")

    def test_the_public_constant_agrees_with_the_resolver(self):
        self.assertEqual(litegrip.__version__, litegrip._detect_version())

    def test_version_is_exported_and_non_empty(self):
        self.assertIn("__version__", litegrip.__all__)
        self.assertTrue(litegrip.__version__)


if __name__ == "__main__":
    unittest.main()
