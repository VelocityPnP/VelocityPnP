#!/usr/bin/env python3
"""Run the installed VelocityPnP model manager from a source checkout."""

from pathlib import Path
import sys


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from velocitypnp.model_cache import main  # noqa: E402


if __name__ == "__main__":
    main()
