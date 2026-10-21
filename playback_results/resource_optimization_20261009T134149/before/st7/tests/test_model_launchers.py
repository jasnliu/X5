"""Exercise the actual launchers without importing audio or opening a device."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('version', ['snare_v2', 'hihat_v2', 'ride_v3'])
@pytest.mark.parametrize('inherited', [None, '', '0'])
def test_launcher_enables_rate_conversion_and_preserves_explicit_override(tmp_path, version, inherited):
    root = tmp_path / 'checkout with spaces'
    launcher = root / 'models' / version / 'run.sh'
    launcher.parent.mkdir(parents=True)
    shutil.copy2(ROOT / 'models' / version / 'run.sh', launcher)
    python = root / '.venv' / 'bin' / 'python'
    python.parent.mkdir(parents=True)
    python.write_text(f'#!{sys.executable}\n'
                      'import json, os, sys\n'
                      'print(json.dumps(dict(cwd=os.getcwd(), args=sys.argv[1:], '
                      'conversion=os.environ.get("PA_ALSA_PLUGHW"))))\n')
    python.chmod(0o755)
    env = os.environ.copy()
    env.pop('PA_ALSA_PLUGHW', None)
    if inherited is not None:
        env['PA_ALSA_PLUGHW'] = inherited
    result = subprocess.run([str(launcher), '--device', 'test microphone'],
                            cwd=tmp_path, env=env, text=True, capture_output=True, check=True)
    launched = json.loads(result.stdout)
    assert launched['cwd'] == str(root)
    assert launched['conversion'] == (inherited or '1')
    assert launched['args'] == ['cymbal.py', '--config', f'models/{version}/config.yaml',
                                'listen', '--device', 'test microphone']
