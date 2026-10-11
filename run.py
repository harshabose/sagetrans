"""Run the whole study: edit the YAML files in configs/, then `python run.py`.

See `python run.py --help`, `python run.py --check`, and configs/README.md.
"""

import sys

from sagetrans.io.cli import main

if __name__ == "__main__":  # the guard matters: worker processes re-import this module on Windows
    sys.exit(main())
