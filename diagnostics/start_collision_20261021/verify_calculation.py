"""Reuse the complete software-only Calculate harness on THIS failing capture."""
from diagnostics.feedback_isolation_20261021 import verify_calculation as verify
from drum_planner.common import ROOT
verify.INCIDENT=ROOT/'planner_runs/20261021T170415Z-baseline-8b9ce3/calculate-2e584558'
if __name__=='__main__':verify.main()
