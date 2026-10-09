First capture stopped before any strike (0 clips), because initial native CSP
centering was still moving during stationary validation. No off-center disable
was sent. After the existing center target settled, independent fresh feedback
proved BOTH centers. release_verified_center.py then released only at center and
query_final_disabled.json confirmed all 16 disabled and fault-free.
Fix: setup now holds the right arm's live pose; only the validated synchronized
stream owns the initial center movement.
