"""Compatibility entry point for the new isolated coverage experiment.

Read README_COVERAGE.md first. Without --fly this is read-only; it never takes off.
The previous implementation is retained in legacy/boustrophedon_sweep.py.
"""
from coverage_mission.runtime import main

if __name__ == '__main__':
    raise SystemExit(main())
