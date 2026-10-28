"""Command line: ``python run.py`` (or ``python -m sagetrans``) runs a whole study.

    python run.py                          # run everything configured in configs/
    python run.py --check                  # validate the inputs and show what would run
    python run.py --config my_configs/     # use another config folder
    python run.py --mode release           # refuse to run unless nothing is TBD or missing
    python run.py --only A D E             # just these analyses
    python run.py --force                  # ignore cached results
    python run.py --fresh-cache            # delete the task cache first, then compute everything
    python run.py --serial                 # one process (easier to debug)
    python run.py --verify                 # also run the test suite and feed the release gate
    python run.py --view [FOLDER]          # open the results as report.html (+ tables.xlsx); no run

Exit codes: 0 ok, 1 a task failed, 2 the inputs are unusable (or release mode refused),
3 release mode finished but the release gate blocked the result.
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path

from sagetrans.io.inputs import InputError, load_inputs
from sagetrans.io.settings import ALL_ANALYSES
from sagetrans.io.study import EXIT_INPUTS, EXIT_OK, preflight, run_study


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="run.py", description="Run the Sage VTOL transition study from a config folder.",
        epilog="Edit the YAML files in the config folder, then run this one command.",
    )
    p.add_argument("--config", default="configs", help="config folder (default: configs)")
    p.add_argument("--out", default=None, help="output folder (default: output_dir in study.yaml)")
    p.add_argument("--mode", choices=("draft", "release"), default=None,
                   help="override study.yaml: draft allows placeholders, release refuses them")
    p.add_argument("--check", action="store_true",
                   help="validate inputs and show the plan, then stop")
    p.add_argument("--only", nargs="+", metavar="ANALYSIS", choices=ALL_ANALYSES,
                   help=f"run only these analyses ({', '.join(ALL_ANALYSES)})")
    p.add_argument("--force", action="store_true", help="ignore cached task results")
    p.add_argument("--fresh-cache", action="store_true",
                   help="delete the task cache of this run first, then compute every task "
                        "(the start of an acceptance run)")
    p.add_argument("--serial", action="store_true", help="run tasks one at a time, in-process")
    p.add_argument("--workers", type=int, default=None, help="number of parallel worker processes")
    p.add_argument("--junit", default=None, help="a pytest JUnit file for the release gate")
    p.add_argument("--verify", action="store_true",
                   help="run the test suite first and feed the release gate (a few minutes)")
    p.add_argument("--view", nargs="?", const="", default=None, metavar="FOLDER",
                   help="view an existing results folder (default: output_dir/run_name of "
                        "study.yaml): writes report.html and tables.xlsx there and opens the page; "
                        "runs nothing")
    p.add_argument("--no-open", action="store_true", help="with --view: do not open the browser")
    return p


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    config = Path(args.config)
    if args.view is not None:
        clash = [f for f, on in (("--only", args.only), ("--force", args.force),
                                 ("--verify", args.verify), ("--check", args.check),
                                 ("--fresh-cache", args.fresh_cache)) if on]
        if clash:
            parser.error(f"--view only shows existing results; it cannot be combined with "
                         f"{', '.join(clash)}")
        from sagetrans.io.view import default_folder, view

        folder = Path(args.view) if args.view else default_folder(config, args.out)
        return view(folder, open_browser=not args.no_open)
    if args.no_open:
        parser.error("--no-open only applies to --view")
    try:
        inp = load_inputs(config, args.mode, set(args.only) if args.only else None)
    except InputError as exc:
        print(f"Cannot use the config in {config}:", file=sys.stderr)
        for prob in exc.problems:
            print(f"  - {prob}", file=sys.stderr)
        return EXIT_INPUTS
    only = {a for a in args.only} if args.only else None
    if args.check:
        root = Path(args.out) if args.out else Path(inp.study.output_dir)
        for line in preflight(inp, root / inp.study.run_name / "_work"):
            print(line)
        if inp.mode == "release" and inp.draft_reasons:
            print("release mode would REFUSE to run (see above).")
            return EXIT_INPUTS
        print("inputs are valid.")
        return EXIT_OK

    outcome = run_study(
        config, mode=args.mode, only=only, force=args.force, fresh_cache=args.fresh_cache,
        workers=1 if args.serial else args.workers, out_dir=args.out, junit=args.junit,
        verify=args.verify,
    )  # fmt: skip
    for m in outcome.messages:
        print(m, file=sys.stderr)
    if outcome.folder is not None:
        print(f"\nresults: {outcome.folder}")
        print(f"  report:       {outcome.folder / 'report.md'}")
        print(f"  run record:   {outcome.folder / 'run_record.json'}  [{outcome.label}]")
        if outcome.gate is not None:
            print(outcome.gate.summary())
        print(f"done in {outcome.seconds / 60:.1f} min")
    return outcome.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
