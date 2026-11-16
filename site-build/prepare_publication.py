"""Reproduce the reviewed HTML exactly; upload only an unreferenced Git blob.

This build-branch helper never edits any branch ref or changes Pages settings.
Historical viewer provenance is retained only after every replay file matches;
new execution evidence is kept separately in the publication artifact.
"""
from __future__ import annotations
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import urllib.request
import zipfile

REPO = 'yongkyuns/navigation_research'
EXPECTED_HTML = 'd210c14d1ef9c5ef8e5f1f38159fa93b6cb569714b2d17cbb9160128cfe8eaed'
EXPECTED_PLAYBACK = 'a398a75a07edba37062aed4cf90171754cf7781d692e528d10b29c2cb0417dca'
SOURCES_SHA = '77c2d1810780668f814825b84c81ce93a50b7bc0c74310fcc0c2b95f0102fe0c'
ARTIFACTS = [
    (14128189886, 'navigation_native_bridge.zip', 'bridge', 'bdcd982f93b454671d751130e26f10463cda1a35d7f37814fa3969e66c7d6c39'),
    (14128164085, 'pinned-fusion-inspection.zip', 'inspection', 'a69d7c903f812d44d3969726abe8b72e11be87c3d442a8941d65082cff6eb9fb'),
    (14167816067, 'navigation_full_replay_latest_ci.zip', 'navigation_full_replay_latest_ci', 'ce381a2b1c44b9b15c70c6ddd26209db678a4f81afe5f4912df26a54e47e8f29'),
    (14165099072, 'navigation_recovery_latest_ci.zip', 'navigation_recovery_latest_ci', 'c8dcfb34c500933e487313ddd420888f62db96259c8255ec1bdc668cd2681b65'),
]
# Intel's versioned download and published SHA-256, checked before extraction.
# https://www.intel.com/content/www/us/en/download/684897/intel-software-development-emulator.html
SDE_URL = 'https://downloadmirror.intel.com/924984/sde-external-10.13.1-2026-07-28-lin.tar.xz'
SDE_SHA256 = '94e97d623fec54385686e1e7ba65ebc9941748c05ee451423948334892bf2b50'
SDE_DIRECTORY = 'sde-external-10.13.1-2026-07-28-lin'
RUNTIME_PROBE = '''import ctypes, hashlib, json, pathlib, platform
import numpy as np
paths = list((pathlib.Path(np.__file__).parent.parent / "numpy.libs").glob("libscipy_openblas64_*.so"))
if len(paths) != 1:
    raise RuntimeError("Expected the pinned NumPy wheel's single OpenBLAS library")
lib = ctypes.CDLL(str(paths[0]))
core = lib.scipy_openblas_get_corename64_
core.restype = ctypes.c_char_p
config = lib.scipy_openblas_get_config64_
config.restype = ctypes.c_char_p
print(json.dumps({"python": platform.python_version(), "numpy": np.__version__,
                  "openblas_core": core().decode(), "openblas_config": config().decode(),
                  "openblas_sha256": hashlib.sha256(paths[0].read_bytes()).hexdigest(),
                  "cpu_features": np._core._multiarray_umath.__cpu_features__}))
'''


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check(path: Path, expected: str) -> None:
    actual = sha(path)
    if actual != expected:
        raise ValueError(f'Hash mismatch: {path.name}: {actual} != {expected}')


def run(*args: object, env: dict[str, str] | None = None) -> None:
    subprocess.run([str(x) for x in args], check=True, env=env)


def numerical_python(root: Path, evidence: Path) -> tuple[list[str], dict[str, str], dict]:
    """Pin SkylakeX natively or via checked SDE; never substitute numeric data.

    Child injection keeps host ptrace/security settings unchanged. SDE lives
    only in temporary build storage, not in the repository, viewer or estimator.
    All existing replay and HTML byte gates remain authoritative.
    """
    host_env = dict(os.environ)
    host_env.pop('OPENBLAS_CORETYPE', None)
    host = json.loads(subprocess.check_output([sys.executable, '-c', RUNTIME_PROBE], env=host_env))
    proof = {'host': host, 'target_openblas_core': 'SkylakeX',
             'security_settings_modified': False, 'mode': None}
    evidence.write_text(json.dumps(proof, indent=2) + '\n')
    env = dict(os.environ, OPENBLAS_CORETYPE='SkylakeX', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    command = [sys.executable]
    if host['cpu_features'].get('AVX512_SKX', False):
        proof['mode'] = 'native-skylakex'
    else:
        archive = root / 'intel-sde.tar.xz'
        with urllib.request.urlopen(SDE_URL, timeout=60) as response, archive.open('wb') as output:
            while chunk := response.read(1024 * 1024):
                output.write(chunk)
        check(archive, SDE_SHA256)
        with tarfile.open(archive) as bundle:
            bundle.extractall(root, filter='data')
        executable = root / SDE_DIRECTORY / 'sde64'
        if not executable.is_file():
            raise FileNotFoundError('Checked Intel SDE archive lacks sde64')
        command = [str(executable), '-skx', '-injection', 'child', '--', sys.executable]
        proof.update(mode='intel-sde-skylakex', sde_url=SDE_URL, sde_sha256=SDE_SHA256,
                     sde_executable_sha256=sha(executable))
    evidence.write_text(json.dumps(proof, indent=2) + '\n')
    target = json.loads(subprocess.check_output([*command, '-c', RUNTIME_PROBE], env=env))
    proof['target'] = target
    evidence.write_text(json.dumps(proof, indent=2) + '\n')
    if (target['python'] != '3.13.5' or target['numpy'] != '2.3.5'
            or target['openblas_core'] != 'SkylakeX'
            or not target['cpu_features'].get('AVX512_SKX', False)):
        raise RuntimeError('Numeric runtime does not match the pinned reviewed target')
    print(json.dumps(proof, indent=2), flush=True)
    return command, env, proof


def extract_zip(path: Path, dest: Path) -> None:
    dest.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path) as archive:
        for member in archive.infolist():
            if not (dest / member.filename).resolve().is_relative_to(dest.resolve()):
                raise ValueError('Unsafe ZIP path')
            if (member.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError('ZIP symlink rejected')
        archive.extractall(dest)


def build(repo: Path, root: Path, archives: Path, download: bool) -> None:
    root.mkdir(parents=True, exist_ok=False)
    archives.mkdir(parents=True, exist_ok=True)
    out = root / 'publication'
    out.mkdir()
    numeric, numeric_env, numeric_proof = numerical_python(root, out / 'numerical-runtime.json')
    work = root / 'work'
    work.mkdir()
    sources = root / 'sources.tar.xz'
    sources.write_bytes(b''.join((repo / 'site-build' / f'source.part-{i:02d}.bin').read_bytes() for i in range(7)))
    check(sources, SOURCES_SHA)
    src = root / 'sources'
    src.mkdir()
    with tarfile.open(sources) as archive:
        archive.extractall(src, filter='data')
    for aid, name, directory, digest in ARTIFACTS:
        path = archives / name
        if download:
            with path.open('wb') as output:
                subprocess.run(['gh', 'api', f'repos/{REPO}/actions/artifacts/{aid}/zip'], stdout=output, check=True)
        check(path, digest)
        extract_zip(path, work / directory)
    inspection = work / 'inspection'
    with tarfile.open(inspection / 'fusion-source-datasets.tar.gz') as archive:
        archive.extractall(inspection, filter='data')
    cache = work / 'native_fusion_recorded_evaluation/native_fusion_evaluation/map_cache'
    cache.mkdir(parents=True)
    shutil.copyfile(repo / 'integration/imu_gnss/map_cache/osm-roads.json.gz', cache / 'osm-roads.json.gz')
    lib = work / 'bridge/libroutefusion_native_bridge.so'
    check(lib, '5ad7955f748154e3ae66ed6fd5f04c0519991aac3ca3298dbcd772d4cd93affd')
    run(*numeric, '-m', 'pytest', '-q', src / 'source/test_centerline_experiment.py', src / 'source/test_persistent_map_ekf.py', f'--junitxml={out / "unit-tests.xml"}', env=numeric_env)
    original = root / 'original'
    run(*numeric, src / 'build_data.py', '--workdir', work, '--archives', archives, '--output', original, env=numeric_env)
    check(original / 'playback_data.json', EXPECTED_PLAYBACK)
    replays = root / 'replays'
    run(*numeric, src / 'source/replay_persistent_map.py', '--playback', original / 'playback_data.json', '--datasets', inspection / 'upstream/web/datasets', '--library', lib, '--source-root', inspection / 'upstream', '--output', replays, env=numeric_env)
    expected = json.loads((src / 'expected-replays.json').read_text())
    for name, digest in expected.items():
        check(replays / name, digest)
    # Preserve fresh-run evidence; restore historical metadata solely to reproduce
    # the exact originally reviewed file, after validating all numeric bytes.
    shutil.copyfile(replays / 'verification.json', out / 'fresh-replay-verification.json')
    shutil.copyfile(src / 'historical-verification.json', replays / 'verification.json')
    run(sys.executable, src / 'source/build_persistent_viewer.py', '--original', original / 'playback_data.json', '--replays', replays, '--output', out / 'index.html')
    check(out / 'index.html', EXPECTED_HTML)
    proof = {
        'html_sha256': EXPECTED_HTML,
        'original_playback_sha256': EXPECTED_PLAYBACK,
        'exact_replay_sha256': expected,
        'source_archive_sha256': SOURCES_SHA,
        'execution_commit': os.environ.get('GITHUB_SHA'),
        'run_id': os.environ.get('GITHUB_RUN_ID'),
        'fresh_native_replays': 28,
        'fresh_native_build': False,
        'historical_metadata_retained_for_byte_identical_html': True,
        'branch_refs_modified_by_helper': False,
        'numerical_runtime': numeric_proof,
    }
    (out / 'publication.json').write_text(json.dumps(proof, indent=2) + '\n')
    print(json.dumps(proof, indent=2), flush=True)


def upload(root: Path) -> None:
    path = root / 'publication/index.html'
    check(path, EXPECTED_HTML)
    data = path.read_bytes()
    expected_blob = hashlib.sha1(f'blob {len(data)}\0'.encode() + data).hexdigest()
    payload = root / 'blob-request.json'
    payload.write_text(json.dumps({'encoding': 'base64', 'content': base64.b64encode(data).decode()}))
    result = json.loads(subprocess.check_output(['gh', 'api', '--method', 'POST', f'repos/{REPO}/git/blobs', '--input', str(payload)]))
    if result['sha'] != expected_blob:
        raise ValueError('GitHub blob hash differs from reviewed HTML')
    payload.unlink()
    target = root / 'publication/publication.json'
    proof = json.loads(target.read_text())
    proof['html_git_blob_sha'] = result['sha']
    target.write_text(json.dumps(proof, indent=2) + '\n')
    print('VERIFIED_HTML_BLOB=' + result['sha'], flush=True)
    print(json.dumps(proof, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=['build', 'upload'])
    parser.add_argument('--repo', type=Path, default=Path.cwd())
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--archives', type=Path)
    parser.add_argument('--download', action='store_true')
    args = parser.parse_args()
    if args.stage == 'build':
        if args.archives is None:
            parser.error('--archives is required for build')
        build(args.repo.resolve(), args.root.resolve(), args.archives.resolve(), args.download)
    else:
        upload(args.root.resolve())
