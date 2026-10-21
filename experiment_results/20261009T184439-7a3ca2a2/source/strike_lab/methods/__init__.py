"""Eight method families, nine selectable modes (two powered curve shapes)."""
from .baseline import DEFINITION as baseline
from .gravity import DEFINITION as gravity, VARIABLE as variable
from .powered import DEFINITION as powered, COSINE as cosine
from .torque import DEFINITION as torque
from .hybrid import DEFINITION as hybrid
from .impedance import DEFINITION as impedance
from .optimized import DEFINITION as optimized

METHODS = {m.id: m for m in (baseline, gravity, variable, powered, cosine,
                            torque, hybrid, impedance, optimized)}
