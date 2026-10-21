"""Offline tempo UI smoke, accommodating the existing async recording preflight.

Run from X5 after sourcing ROS and install/setup.bash. This does NOT change the
original test script, does NOT open physical CAN/ESP32, and redirects screenshots.
"""
from pathlib import Path

root = Path(__file__).resolve().parents[2]
path = root / 'tests/beat_tempo_ui_check.py'
source = path.read_text()
check = '        assert str(app.start_button.cget("state")) == "normal"'
assert source.count(check) == 1
wait = """        deadline = time.monotonic() + 30.
        while str(app.start_button.cget("state")) != "normal" and time.monotonic() < deadline:
            app.root.update()
            time.sleep(.01)
"""
source = source.replace(check, wait + check)
source = source.replace("ROOT/'diagnostics/beat_tempo_20261005'/name",
                        "ROOT/'diagnostics/bass_drum_20261010'/name")
exec(compile(source, str(path), 'exec'), {'__file__': str(path), '__name__': '__main__'})
