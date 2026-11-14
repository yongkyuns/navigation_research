"""Safely rerun tests, all 60 scenarios, and six replays without changing the snapshot."""
import argparse
import json
from pathlib import Path
from evidence import ROOT, evaluate
from verify_snapshot import verify


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'runs/reproduction')
    args = parser.parse_args()
    print(json.dumps(verify(), indent=2))
    evaluate(args.output)
    print(json.dumps(verify(), indent=2))


if __name__ == '__main__':
    main()
