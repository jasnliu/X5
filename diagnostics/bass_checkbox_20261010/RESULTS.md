# Optional swing bass checkbox

Added `Use bass drum (motor 1) in swing beat` beneath Beat BPM in the
start_beat.sh hardware swing UI (and offline preview). It defaults checked,
preserving existing behavior. Unchecked omits the bass scheduler and all K
strike commands while leaving the hi-hat scheduler, ride and snare unchanged.
The setup choice is committed before RUN and locked until Center + Relax.
Simulation, manual hardware-test and recording-only modes do not expose an
inapplicable bass checkbox. Header/status and timeline diagnostics reflect it.

Validation: 106 focused offline tests passed. A real Tk hardware-mode UI test
with all physical I/O forbidden/mocked verified default ON, toggle OFF, tempo
changes, active-phase lockout and re-enabling after relaxation. A serial PTY
workflow test verified exactly OB repeated four times with bass off versus
OKBK repeated four times with bass on. A partial PTY read in the first new test
was fixed by collecting the complete bounded expected byte sequence; production
serial code was not changed. Screenshot: checkbox_off.png (mocked UI only).

Syntax checks passed. All 482 protected files are byte-identical, including
X5data, saved recordings, configuration, bass collector, firmware, pulse code,
launchers and left return/dual recording code. No physical hardware was operated;
this is not live motor validation. No training or additional data collection.
