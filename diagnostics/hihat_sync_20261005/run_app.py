"""Physical evidence wrapper: ordinary app, only acceptance duration is longer."""
import functools
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import camera_playback.beat_evidence as evidence
# Production workflow/controllers/detectors are not replaced or assisted.
evidence.BeatEvidence = functools.partial(evidence.BeatEvidence, seconds=40.)
from camera_playback.app import main
if __name__ == '__main__':main()
