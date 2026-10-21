# ESP32 hi-hat and bass-drum firmware used by camera playback

This is the proven `motor_beat` firmware copied from
`/home/jason/Proyectos3/esp-main`. The integrated host is now
`camera_playback/hihat.py`; `start_beat.sh` opens the connected
CP2102 ESP32 at 115200 baud when launched with `--hardware`.
`--hardwaretest` performs isolated, user-requested arm strikes without a swing
beat and does not open the ESP32 serial port.

Normal hardware startup automatically calibrates before ride search. The host
verifies calibration-v2 support using `Q`; old firmware cannot silently continue
at its fixed angle. As soon as the controller and detector are ready, it tries
90..115° in 5° steps, with no ntfy alert or countdown, holding closed for 2 s
and open for 2 s. The first hi-hat detector HIT
selects the swing target; no HIT at 115° is a fault.

The protocol is:

- `A90\n` through `A115\n`: set the integer calibrated angle, only while released
  at open zero; acknowledged by `ANGLE degrees=... counts=...`. Does not move.
- `B`: motor 2 closes to and holds the acknowledged calibrated angle;
- `C` or `J`: legacy fixed **100-degree** closure (unchanged for collectors/keyboard);
- `O`: motor 2 returns to encoder zero, then releases;
- `K`: motor 1 bass strike, clockwise **PWM 180/255 at 20 kHz for 50 ms**,
  then both enable pins go LOW and the motor coasts/releases. This is the exact
  `esp-main/run_keys.sh` F-key strike, unchanged in this firmware;
- `Q`: read protocol bounds, configured angle, encoder position, active target,
  released state, zero-return arrival latch (`opened`) and fault state, with no motion;
- `H`: 100 ms watchdog heartbeat;
- `S`: release both firmware motor outputs when the program exits.

The beat program sends `O` on beats 1/3 and `B` on beats 2/4, shifted together
by its automatically tuned hi-hat advance. This changes timing, not the learned
rotation. During normal hardware swing, the host also sends `K` on **every
quarter note (1, 2, 3, 4)** at the user's BPM. Bass starts at beat 1 after the
ride pickup, never during calibration or on pickup/triplet extras. It has no
calibration or timing offset and does not inherit the hi-hat advance. Right-arm
joint 7 continues to play the ride cymbal. Both motors share the existing
serial owner; no second port or host-side pulse timer is needed.

Normal stop cancels future kicks and returns motor 2 to zero; any current bass
pulse ends automatically after 50 ms. Emergency Relax/exit sends `S`, releasing
both motors. Late commands trigger controlled stop rather than catch-up hits.
Collectors, simulation, manual hardware-test and recording-only modes do not
add bass strikes. Existing X5 calibration-v2 firmware already supports this
unchanged `K` pulse, so the bass addition itself requires no reflash.

All calibrated angle commands are bounded to **90..115°** in firmware and host.
Travel retains encoder quantization and closed-loop tolerance; 115° is the
maximum commanded target, not a claim of zero physical overshoot. The firmware
reports counts (115° = 172 counts). The movement timeout is 2 seconds; motor
faults latch until reset and cannot be cleared by another closure command.
The 400 ms heartbeat watchdog and original motor gains remain active. Startup
always assumes the hi-hat is physically open at its encoder-zero position.
Open completion records the actual zero crossing before coasting/releasing.
The host waits for the released encoder to settle, rather than repeatedly
reopening a motor that already returned but passively coasted beyond zero.

The copied `flash_beat.sh` is provided only when the board needs its firmware
reloaded. Normal camera playback does **not** flash the ESP32. Do not run the
standalone `esp-main/motor_beat.py` at the same time because only one process
should own the serial port.
