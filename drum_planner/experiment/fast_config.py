"""Current experiment menu, separate from retained historical method policies."""
LABEL='Fast workspace — fresh paths + sparse CPU timing'
METHODS={'workspace_curve':LABEL}
DEFAULT_METHOD='workspace_curve'
DESCRIPTIONS={'workspace_curve':
    'Fresh broad workspace curves with bounded sparse CPU timing. No CPU-reference solve or result-cache reuse. '
    'Continuous motion limits and dense collision/zone checks; explicit checked-clock fallback if sparse refinement fails. '
    'Software preview only; hardware planners are unchanged.'}
