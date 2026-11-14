"""One-time deterministic import of the original research evidence.

This is NOT a new benchmark. Historical timing/environment strings are restored
from provenance/original-artifacts.json, never reported as current measurements.
Every restored project byte must match the original aggregate SHA-256 manifest.
New experiments belong in runs/ and must never rewrite the frozen snapshot.
"""
from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import zipfile
import numpy as np
from plot_evaluation import plot

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'routefusion_prototype'


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def summary_from(metrics: list[dict]) -> dict:
    summary = {}
    for name in dict.fromkeys(m['scenario'] for m in metrics):
        group = [m for m in metrics if m['scenario'] == name]
        summary[name] = {'runs': len(group)}
        for key in group[0]:
            vals = [m[key] for m in group
                    if isinstance(m[key], (int, float)) and key != 'seed']
            if vals:
                summary[name][key] = {'mean': float(np.mean(vals)),
                                     'min': float(np.min(vals)),
                                     'max': float(np.max(vals))}
    return summary


def manifest(directory: Path) -> bytes:
    return ''.join(f'{digest(p.read_bytes())}  {p.relative_to(directory).as_posix()}\n'
                   for p in sorted(directory.rglob('*'))
                   if p.is_file() and p.name != 'SHA256SUMS.txt').encode()


def run(command: list[str], *, log: Path | None = None) -> None:
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               MPLBACKEND='Agg', PYTHONDONTWRITEBYTECODE='1')
    result = subprocess.run(command, cwd=PROJECT, env=env, check=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if log is not None:
        log.write_bytes(result.stdout)
    print(result.stdout.decode(), end='')


def main() -> None:
    meta = json.loads((ROOT / 'provenance/original-artifacts.json').read_text())
    if (PROJECT / 'results').exists():
        raise SystemExit('Frozen results already exist; use scripts/reproduce.py for reruns.')
    fresh = ROOT / 'runs/import-verification'
    fresh.mkdir(parents=True, exist_ok=False)
    run([sys.executable, '-m', 'pytest', '-q'], log=fresh / 'tests-current.txt')
    run([sys.executable, '-m', 'routefusion.simulate', '--seeds', '10',
         '--start-seed', '100', '--save-traces', '--output', str(fresh / 'results')],
        log=fresh / 'simulation-current.txt')
    with tempfile.TemporaryDirectory(prefix='routefusion-import-') as tmp:
        stage = Path(tmp) / 'routefusion_prototype'
        stage.mkdir()
        sources = [PROJECT / name for name in
                   ['README.md', 'pyproject.toml', 'requirements-tested.txt', 'sources.json']]
        for pattern in ['docs/*.md', 'docs/*.mmd', 'routefusion/*.py', 'tests/*.py']:
            sources.extend(PROJECT.glob(pattern))
        for source in sources:
            destination = stage / source.relative_to(PROJECT)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
        results = stage / 'results'
        shutil.copytree(fresh / 'results', results)
        metrics = json.loads((results / 'metrics.json').read_text())
        for measurement in metrics:
            # Preserve CAPTURED original timings; current timings stay in runs/.
            key = f"{measurement['scenario']}:{measurement['seed']}"
            measurement['runtime_s'] = meta['recorded_runtime_s'][key]
        (results / 'metrics.json').write_bytes(json.dumps(metrics, indent=2, allow_nan=False).encode())
        (results / 'summary.json').write_bytes(json.dumps(summary_from(metrics), indent=2, allow_nan=False).encode())
        (results / 'environment.json').write_bytes(meta['historical_environment_text'].encode())
        (results / 'tests.txt').write_bytes(meta['historical_tests_text'].encode())
        shutil.copyfile(fresh / 'simulation-current.txt', results / 'evaluation_console.txt')
        run([sys.executable, '-m', 'routefusion.replay', str(results / 'parallel_bias_example'),
             '--output', str(results / 'replay_check.jsonl')])
        plot(results / 'summary.json', results / 'figures/synthetic_evaluation.png')
        listing = manifest(stage)
        actual = digest(listing)
        if actual != meta['prototype_manifest_sha256']:
            (fresh / 'candidate-SHA256SUMS.txt').write_bytes(listing)
            raise SystemExit(f'Original snapshot mismatch: {actual}; no frozen files installed.')
        (stage / 'SHA256SUMS.txt').write_bytes(listing)
        count = len([p for p in stage.rglob('*') if p.is_file()])
        if count != meta['prototype_file_count']:
            raise SystemExit('Original file count mismatch')
        design = (stage / 'docs/DESIGN.md').read_text().replace(
            '`../sources.json`.', '`sources.json` in the accompanying prototype archive.')
        report = '\n\n---\n\n'.join([design.rstrip('\n'), (stage / 'docs/VALIDATION.md').read_text(),
                                      (stage / 'docs/INTEGRATION.md').read_text()]).encode()
        if digest(report) != meta['report_sha256']:
            raise SystemExit('Combined report mismatch')
        shutil.copytree(results, PROJECT / 'results')
        shutil.copyfile(stage / 'SHA256SUMS.txt', PROJECT / 'SHA256SUMS.txt')
        (ROOT / 'routefusion_research_and_design.md').write_bytes(report)
        artifacts = ROOT / 'artifacts'
        artifacts.mkdir(exist_ok=True)
        archive = artifacts / 'routefusion_prototype.zip'
        with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as z:
            for source in sorted(stage.rglob('*')):
                if source.is_file():
                    info = zipfile.ZipInfo('routefusion_prototype/' + source.relative_to(stage).as_posix(),
                                           date_time=(2000, 1, 1, 0, 0, 0))
                    info.compress_type = zipfile.ZIP_DEFLATED
                    info.external_attr = 0o100644 << 16
                    z.writestr(info, source.read_bytes())
        verification = {
            'original_project_files_verified': count,
            'original_manifest_sha256': actual,
            'combined_report_sha256': digest(report),
            'repacked_archive_sha256': digest(archive.read_bytes()),
            'repacked_archive_note': 'New ZIP container; every contained file matches the original snapshot. Original ZIP hash is in original-artifacts.json.',
            'historical_timing_note': 'Snapshot timing fields were restored from captured metadata, not measured again. Fresh verification timings are separate under runs/import-verification.',
            'synthetic_only': True,
        }
        (ROOT / 'provenance/IMPORT_VERIFICATION.json').write_text(json.dumps(verification, indent=2) + '\n')
        print(json.dumps(verification, indent=2))


if __name__ == '__main__':
    main()
