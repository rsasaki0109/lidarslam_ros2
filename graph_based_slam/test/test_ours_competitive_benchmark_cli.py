"""Source-checkout CLI smoke tests for the in-workspace competitor runner."""

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]


def test_source_checkout_cli_help_resolves_package_surface():
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / 'scripts/run_ours_competitive_benchmark.py'),
            '--help',
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert '--runs' in result.stdout
    assert '--bag' in result.stdout

