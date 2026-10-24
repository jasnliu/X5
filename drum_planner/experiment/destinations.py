"""Experiment-only destination discovery and immutable selection snapshots."""
import hashlib
import re
import uuid
from pathlib import Path

from ..common import ROOT, arm_side, digest, endpoint, goal_recording


def files(side, root=ROOT/'destinations'):
    folder=Path(root)/arm_side(side)
    def natural(path):
        return [(1,int(part)) if part.isdigit() else (0,part.casefold())
                for part in re.split(r'(\d+)',path.name)]
    return sorted((p.resolve() for p in folder.glob('*')
                   if p.is_file() and p.suffix.lower()=='.json'),key=natural)


def freeze(path, side, session):
    """Bind the chosen bytes, not a file that could change during calculation."""
    path=Path(path).resolve()
    data=path.read_bytes()
    folder=Path(session)/'selected_destinations'/uuid.uuid4().hex[:12]
    folder.mkdir(parents=True)
    saved=folder/path.name
    saved.write_bytes(data)
    try:
        endpoint(saved,side=side)  # existing schema/model/arm/finite-angle checks
    except Exception:
        saved.unlink();folder.rmdir()
        raise
    return dict(goal_recording=str(saved.resolve()),destination_source=str(path),
                destination_sha256=hashlib.sha256(data).hexdigest())


def goal(context):
    """Legacy/headless contexts retain their existing endpoint default."""
    side=arm_side(context.get('moving_side','right'))
    path=Path(context.get('goal_recording') or goal_recording(side))
    expected=context.get('destination_sha256')
    if expected is not None and digest(path)!=expected:
        raise ValueError('Selected destination snapshot changed; select the destination again')
    return path,endpoint(path,side=side).tolist()
