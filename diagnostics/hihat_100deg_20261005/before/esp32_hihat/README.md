# ESP32 hi-hat firmware used by right camera playback

This is the proven `motor_beat` firmware copied from
`/home/jason/Proyectos3/esp-main`. The integrated host is now
`camera_playback/hihat.py`; `start_beat.sh` opens the connected
CP2102 ESP32 at 115200 baud when launched with `--hardware`.
`--hardwaretest` performs isolated, user-requested arm strikes without a swing
beat and does not open the ESP32 serial port.

During the robot arm's learned 100 BPM loop, the host sends only:

- `C`: motor 2 closes to and holds **110 degrees**;
- `O`: motor 2 returns to encoder zero, then releases;
- `H`: 100 ms watchdog heartbeat;
- `S`: release both firmware motor outputs when the program exits.

The first arm beat sends `C`, the next sends `O`, and this alternates so the
hi-hat strikes every other beat. The host never sends `K`; firmware motor 1 is
replaced rhythmically by right-arm joint 7 and remains released.

The copied `flash_beat.sh` is provided only when the board needs its firmware
reloaded. Normal camera playback does **not** flash the ESP32. Do not run the
standalone `esp-main/motor_beat.py` at the same time because only one process
should own the serial port.
