"""CI helper: check the verification tests T-01 to T-15 from a pytest JUnit file.

    pytest --junitxml=junit.xml
    python -m sagetrans.io.ci_check junit.xml

Exits non-zero if a required verification ID has no test or a failing test. This is what feeds
`gate.release_gate` its `verification` mapping in CI, and it catches a renamed or deleted
verification test (an ID with no test counts as a failure).
"""

from __future__ import annotations

import sys
from collections.abc import Sequence
from pathlib import Path

from sagetrans.io.gate import REQUIRED_TESTS, verification_from_junit


def check(junit: Path, required: Sequence[str] = REQUIRED_TESTS) -> tuple[bool, list[str]]:
    """(all required IDs present and passing, one report line per ID)."""
    result = verification_from_junit(junit)
    ok = True
    lines: list[str] = []
    for tid in required:
        if tid not in result:
            state, ok = "MISSING", False
        elif not result[tid]:
            state, ok = "FAILED", False
        else:
            state = "pass"
        lines.append(f"{tid}: {state}")
    return ok, lines


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) != 1:
        print("usage: python -m sagetrans.io.ci_check <junit.xml>", file=sys.stderr)
        return 2
    ok, lines = check(Path(args[0]))
    print("\n".join(lines))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
