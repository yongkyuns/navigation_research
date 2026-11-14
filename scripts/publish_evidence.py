"""One-time publication of original records and separately labelled fresh evidence."""
from __future__ import annotations
import json
import os
from pathlib import Path
import shutil
import zipfile
from evidence import ROOT, PROJECT, sha, original_records, check_sources, evaluate


def main() -> None:
    if (PROJECT / 'results').exists():
        raise SystemExit('Published results already exist; use reproduce.py for a new run')
    ref, records = original_records()
    # Repair the sole text-import discrepancy, but only to the captured source hash.
    source = PROJECT / 'sources.json'
    data = source.read_bytes()
    expected = ref['source_sha256']['sources.json']
    if sha(data) != expected and sha(data + b'\n') == expected:
        source.write_bytes(data + b'\n')
    check_sources()
    for name, data in records.items():
        (ROOT / 'provenance' / name).write_bytes(data)
    metadata = json.loads((ROOT / 'provenance/original-artifacts.json').read_text())
    design = (PROJECT / 'docs/DESIGN.md').read_text().replace(
        '`../sources.json`.', '`sources.json` in the accompanying prototype archive.')
    report = '\n\n---\n\n'.join([design.rstrip('\n'), (PROJECT / 'docs/VALIDATION.md').read_text(),
                                  (PROJECT / 'docs/INTEGRATION.md').read_text()]).encode()
    if sha(report) != metadata['report_sha256']:
        raise ValueError('Standalone original report checksum differs')
    fresh = ROOT / 'runs/import-verification'
    verification = evaluate(fresh)
    shutil.copytree(fresh / 'results', PROJECT / 'results')
    (PROJECT / 'results/README.md').write_text(
        '# Published reproduction evidence\n\n'
        'These are fresh synthetic runs of the unchanged original code, not byte-identical '
        'copies of the original chat traces. All 60 non-timing metric records and the '
        'summary match the preserved originals within 1e-9 absolute/relative tolerance. '
        'Runtime, environment, test logs, replay files, tiles and the figure were produced '
        'during this publication. No historical timings were substituted.\n\n'
        'The exact original numerical records are in ../../provenance/original-metrics.json '
        'and original-summary.json; the original report/code are checksum-verified. '
        'The historical project README and validation report describe the original study. '
        'Use the root README for safe, non-overwriting reruns.\n\n'
        'Six replay cases (720 outputs) and the original 24 algorithm tests are checked. '
        'No hardware or real-city dataset evaluation is claimed.\n')
    files = sorted(p for p in PROJECT.rglob('*') if p.is_file()
                   and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts
                   and not any(part.endswith('.egg-info') for part in p.parts)
                   and p.name != 'SHA256SUMS.txt')
    listing = ''.join(f'{sha(p.read_bytes())}  {p.relative_to(PROJECT).as_posix()}\n' for p in files).encode()
    (PROJECT / 'SHA256SUMS.txt').write_bytes(listing)
    (ROOT / 'routefusion_research_and_design.md').write_bytes(report)
    artifacts = ROOT / 'artifacts'
    artifacts.mkdir(exist_ok=True)
    archive = artifacts / 'routefusion_prototype.zip'
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as z:
        for p in sorted(files + [PROJECT / 'SHA256SUMS.txt']):
            info = zipfile.ZipInfo('routefusion_prototype/' + p.relative_to(PROJECT).as_posix(),
                                   date_time=(2000, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            z.writestr(info, p.read_bytes())
    record = dict(verification, published_project_files=len(files) + 1,
                  published_manifest_sha256=sha(listing), combined_report_sha256=sha(report),
                  published_archive_sha256=sha(archive.read_bytes()),
                  published_chart_sha256=sha((PROJECT / 'results/figures/synthetic_evaluation.png').read_bytes()),
                  original_archive_byte_identical=False,
                  publication_note='Original code/report/numerical records preserved; new traces, logs and ZIP explicitly labelled as reproduction.',
                  github_run_id=os.environ.get('GITHUB_RUN_ID'), github_sha=os.environ.get('GITHUB_SHA'))
    (ROOT / 'provenance/IMPORT_VERIFICATION.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main()
