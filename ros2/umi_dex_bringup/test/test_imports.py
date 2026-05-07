"""Smoke tests: importability + launch-file sanity.

Scoped intentionally tiny — their purpose is to flip `colcon test` green
(the bare package had no tests, returning NO TESTS RAN). Real behavioral
coverage lives under the top-level `tests/` dir for the Python pipeline.
"""

import importlib
import os

import pytest


NODE_MODULES = [
    "umi_dex_bringup.can_raw_node",
    "umi_dex_bringup.usart_raw_node",
    "umi_dex_bringup.interactive_capture_node",
]


@pytest.mark.parametrize("modname", NODE_MODULES)
def test_node_imports(modname: str) -> None:
    importlib.import_module(modname)


def test_simple_launches_generate() -> None:
    """d455 / d405 launches are pure FindPackageShare substitutions — safe to
    evaluate without a populated share dir. `capture.launch.py` is skipped
    here because it reads `camera_serials.conf` from the share dir at
    generate-time, which is test-environment dependent."""
    import importlib.util

    here = os.path.dirname(__file__)
    launch_dir = os.path.abspath(os.path.join(here, "..", "launch"))

    for fname in ("d455.launch.py", "d405.launch.py"):
        spec = importlib.util.spec_from_file_location(
            fname.replace(".", "_"), os.path.join(launch_dir, fname)
        )
        assert spec and spec.loader
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ld = mod.generate_launch_description()
        assert ld is not None
        assert len(ld.entities) > 0
