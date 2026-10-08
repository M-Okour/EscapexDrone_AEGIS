import importlib.util
import os

LAUNCH_FILE = os.path.join(os.path.dirname(__file__), '..', 'launch', 'exd.launch.py')


def test_launch_file_loads():
    spec = importlib.util.spec_from_file_location('exd_launch', LAUNCH_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()
    assert len(ld.entities) > 5
