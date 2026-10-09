# Return-fault software fix — OFFLINE verification

- 680 unit/regression tests passed on the final source (`full_tests.txt`).
- The 1.5 s ESP32 freshness threshold is unchanged; real stale data still faults.
- Virtual serial firmware test: heartbeats/status continue during a 1.8 s GUI pause.
- Injected hi-hat fault during MIT return: right-arm targets and mocked poses still match the original reference; return completes.
- Normal CSP and MIT return command traces match start_beatTest's saved reference traces exactly (208/343 movement frames).
- Actual Tk emergency button invoked with fake sockets: 24 disable frames per arm, no centering/enable commands.
- Regression cases cover interrupted wrist-goal recovery, cleanup errors, terminated writers, repeated close, and communication-loss shutdown.
- Reference runtime, launcher and recordings: 289 protected file hashes unchanged.
- No physical arm/device trial performed. Command/pose equivalence here uses simulated feedback and does not establish real-world clearance or disable delivery.

## Stop behavior

Normal Center + Relax retains per-arm arrival checks. Emergency Relax/Escape stops writers then attempts disable without centering; restart is required. First close requests centered shutdown; a repeated close/Ctrl-C or the 35 s grace deadline requests emergency disable and exit. Physical power cutoff remains necessary if delivery cannot be confirmed. Torque release can let an unsupported arm fall.

## Test-run notes

An old serial-controller mock needed its new service_running=False field specified. A test run made while editing source correctly triggered the cache's Code changed since import guard. Both were rerun successfully with the final source frozen; cache semantics were not weakened.
