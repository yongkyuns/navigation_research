"""Verify the frozen original project, standalone report, chart and ZIP entries."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path, PurePosixPath
import zipfile

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'routefusion_prototype'


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def verify() -> dict:
    meta = json.loads((ROOT / 'provenance/original-artifacts.json').read_text())
    record = json.loads((ROOT / 'provenance/IMPORT_VERIFICATION.json').read_text())
    listing = (PROJECT / 'SHA256SUMS.txt').read_bytes()
    if sha(listing) != meta['prototype_manifest_sha256']:
        raise ValueError('Original aggregate manifest changed')
    expected = {}
    for line in listing.decode().splitlines():
        digest, name = line.split('  ', 1)
        path = PurePosixPath(name)
        if path.is_absolute() or '..' in path.parts or name in expected:
            raise ValueError(f'Unsafe/duplicate manifest entry: {name}')
        data = (PROJECT / name).read_bytes()
        if sha(data) != digest:
            raise ValueError(f'Frozen file changed: {name}')
        expected[name] = digest
    expected['SHA256SUMS.txt'] = sha(listing)
    if len(expected) != meta['prototype_file_count']:
        raise ValueError('Unexpected original file count')
    checks = {
        ROOT / 'routefusion_research_and_design.md': meta['report_sha256'],
        PROJECT / 'results/figures/synthetic_evaluation.png': meta['chart_sha256'],
        ROOT / 'artifacts/routefusion_prototype.zip': record['repacked_archive_sha256'],
    }
    for path, digest in checks.items():
        if sha(path.read_bytes()) != digest:
            raise ValueError(f'Artifact changed: {path.relative_to(ROOT)}')
    with zipfile.ZipFile(ROOT / 'artifacts/routefusion_prototype.zip') as archive:
        entries = archive.namelist()
        names = {'routefusion_prototype/' + name for name in expected}
        if len(entries) != len(names) or set(entries) != names:
            raise ValueError('Archive entries differ from the frozen project')
        for name, digest in expected.items():
            if sha(archive.read('routefusion_prototype/' + name)) != digest:
                raise ValueError(f'Archive entry changed: {name}')
    return {'verified_project_files': len(expected), 'report': 'verified',
            'chart': 'verified', 'archive_contents': 'verified', 'synthetic_only': True}


if __name__ == '__main__':
    print(json.dumps(verify(), indent=2))
