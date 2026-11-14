"""Verify published bytes and preserved historical records without conflating them."""
from __future__ import annotations
import json
from pathlib import PurePosixPath
import zipfile
from evidence import ROOT, PROJECT, sha, original_records, check_sources, compare_metrics, compare_summary


def verify() -> dict:
    sources = check_sources()
    _, records = original_records()
    for name, data in records.items():
        if (ROOT / 'provenance' / name).read_bytes() != data:
            raise ValueError(f'Historical record changed: {name}')
    record = json.loads((ROOT / 'provenance/IMPORT_VERIFICATION.json').read_text())
    metadata = json.loads((ROOT / 'provenance/original-artifacts.json').read_text())
    listing = (PROJECT / 'SHA256SUMS.txt').read_bytes()
    if sha(listing) != record['published_manifest_sha256']:
        raise ValueError('Published manifest changed')
    expected = {}
    for line in listing.decode().splitlines():
        digest, name = line.split('  ', 1)
        p = PurePosixPath(name)
        if p.is_absolute() or '..' in p.parts or name in expected:
            raise ValueError('Unsafe or duplicate manifest entry')
        if sha((PROJECT / name).read_bytes()) != digest:
            raise ValueError(f'Published file changed: {name}')
        expected[name] = digest
    expected['SHA256SUMS.txt'] = sha(listing)
    if len(expected) != record['published_project_files']:
        raise ValueError('Published file count differs')
    checks = {
        ROOT / 'routefusion_research_and_design.md': metadata['report_sha256'],
        PROJECT / 'results/figures/synthetic_evaluation.png': record['published_chart_sha256'],
        ROOT / 'artifacts/routefusion_prototype.zip': record['published_archive_sha256'],
    }
    for path, digest in checks.items():
        if sha(path.read_bytes()) != digest:
            raise ValueError(f'Artifact changed: {path.relative_to(ROOT)}')
    with zipfile.ZipFile(ROOT / 'artifacts/routefusion_prototype.zip') as z:
        names = {'routefusion_prototype/' + name for name in expected}
        if len(z.namelist()) != len(names) or set(z.namelist()) != names:
            raise ValueError('Archive entry set differs')
        for name, digest in expected.items():
            if sha(z.read('routefusion_prototype/' + name)) != digest:
                raise ValueError(f'Archive content changed: {name}')
    diagnostics = compare_metrics(json.loads((PROJECT / 'results/metrics.json').read_text()),
                                  json.loads(records['original-metrics.json']), PROJECT / 'results')
    compare_summary(json.loads((PROJECT / 'results/summary.json').read_text()),
                    json.loads(records['original-summary.json']), diagnostics)
    return {'verified_published_project_files': len(expected), 'original_source_files': sources,
            'original_numeric_records': len(records), 'original_report': 'checksum verified',
            'published_archive_contents': 'checksum verified',
            'primary_navigation_metrics_match_original': True,
            'tie_sensitive_baseline_diagnostics': diagnostics, 'synthetic_only': True}


if __name__ == '__main__':
    print(json.dumps(verify(), indent=2))
