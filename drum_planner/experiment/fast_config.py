"""Current experiment menu, separate from retained historical method policies."""
LABEL='Workspace curve — configurable joint timing'
METHODS={'workspace_curve':LABEL}
DEFAULT_METHOD='workspace_curve'
DESCRIPTIONS={'workspace_curve':
    'Broad workspace curves with bounded-spline/wrist-guided recovery when the original proposals fail. '
    'Per-joint velocity, acceleration and jerk limits; adjustable timing effort. '
    'No CPU-reference solve or result-cache reuse. '
    'Continuous motion limits and dense collision/zone checks; explicit checked-clock fallback if sparse refinement fails. '
    'Software preview only; hardware planners are unchanged.'}
