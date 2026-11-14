"""Run recorded-data evaluation without overwriting earlier research evidence."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
IDS = ['urban-short-turn-loop-nominal-001', 'urban-short-turn-loop-nominal-002',
       'urban-stop-go-tight-turns-nominal-001', 'ios-mixed-road-drive-001']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--map-cache', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    dataset = args.upstream.resolve() / 'web/datasets'
    library = args.library.resolve()
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               ROUTEFUSION_DATASETS=str(dataset), ROUTEFUSION_NATIVE_LIBRARY=str(library))
    repo = HERE.parents[1]
    prototype = repo / 'routefusion_prototype'
    if not prototype.exists():
        raise ValueError('Run this helper from the checked-out navigation_research repository')
    env['PYTHONPATH'] = os.pathsep.join([str(prototype), str(HERE)])

    def run(name, arguments):
        command = [sys.executable, *map(str, arguments)]
        completed = subprocess.run(command, env=env, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        (out / (name + '.log')).write_text(completed.stdout)
        print(completed.stdout, end='', flush=True)
        if completed.returncode:
            raise RuntimeError(f'{name} failed; preserved log: {out / (name + ".log")}')

    run('integration-tests', ['-m', 'pytest', '-q', HERE / 'tests', '--junitxml', out / 'integration-tests.xml'])
    run('baseline', [HERE / 'evaluate_baseline.py', '--library', library,
                    '--datasets', dataset, '--output', out / 'baseline'])
    common = ['--library', library, '--datasets', dataset,
              '--map', args.map_cache.resolve() / 'osm-roads.json.gz',
              '--map-provenance', args.map_cache.resolve() / 'provenance.json']
    run('road-map', [HERE / 'evaluate_map.py', *common, '--output', out / 'map', '--ids', *IDS])
    for mode, suffix in [('outage30', 'outage30'), ('north_bias20', 'bias20')]:
        run('stress-' + suffix, [HERE / 'evaluate_map.py', *common, '--output', out / ('stress-' + suffix),
                                 '--ids', IDS[1], '--stress', mode])
    run('report', [HERE / 'report.py', '--results', out, '--datasets', dataset, '--output', args.report])
    (out / 'native-library-sha256.json').write_text(json.dumps({
        'sha256': hashlib.sha256(library.read_bytes()).hexdigest(),
        'note': 'Linux shared library is retained in the CI artifact, not required in source control.'}, indent=2) + '\n')
    for csv_path in out.rglob('*.csv'):
        csv_path.with_suffix('.csv.gz').write_bytes(gzip.compress(csv_path.read_bytes(), mtime=0))
        csv_path.unlink()
    checksums = {p.relative_to(out).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in sorted(out.rglob('*')) if p.is_file()}
    (out / 'SHA256SUMS.json').write_text(json.dumps(checksums, indent=2) + '\n')


if __name__ == '__main__':
    main()
