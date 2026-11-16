"""Evidence identity, deterministic build and reject-corruption regression tests."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import re
import zipfile
import pytest

SPEC=importlib.util.spec_from_file_location('builder',Path(__file__).parents[1]/'build.py')
build=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(build)

@pytest.fixture(scope='session')
def artifact():
    p=Path(os.environ['CORRIDOR_ARTIFACT'])
    assert p.is_file()
    return p

@pytest.fixture(scope='session')
def data(artifact):return build.load_evidence(artifact)


def test_exact_recordings_and_each_numerical_value(artifact,data):
    with zipfile.ZipFile(artifact) as z:
        seeds=json.loads(z.read('seeds/playback-data.json'))
        assert data['roads']==seeds['roads']
        assert len(data['roads'])==7333
        for d in data['datasets']:
            original=json.loads(z.read('first/'+d['id']+'.json'))
            assert d['rows']==original['rows']
            assert d['diagnostics']==original['diagnostics']
            seed=next(s for s in seeds['datasets'] if s['id']==d['id'])
            assert d['gnss']==seed['gnss'] and d['input_sha256']==seed['input_sha256']


def test_no_fabricated_positions_or_fallback(data):
    total=0;gaps=0
    for d in data['datasets']:
        for r in d['rows']:
            if r['raw_position'] is None:
                assert all(r['outputs'][m]['position'] is None for m in build.MODELS)
            else:
                total+=1
                assert all(r['outputs'][m]['ready'] for m in build.MODELS)
                if r['original_edge'] is None and r['outputs']['correlated']['map_updates']>0:
                    gaps+=1
                    assert r['outputs']['correlated']['position'] is not None
    assert total==1402 and gaps>0


def test_all_27_member_hashes_are_retained(data):
    assert len(data['provenance']['member_sha256'])==27
    assert data['provenance']['ci_head']==build.HEAD
    assert data['provenance']['sigma_m']==.25
    assert data['provenance']['map_sigma_m']==1.5


def test_archive_sha_guard(artifact,tmp_path):
    p=tmp_path/'bad.zip';p.write_bytes(artifact.read_bytes()+b'changed')
    with pytest.raises(ValueError,match='reviewed'):build.load_evidence(p)


def altered(artifact,tmp_path,change):
    with zipfile.ZipFile(artifact) as z:members={n:z.read(n) for n in z.namelist()}
    change(members)
    p=tmp_path/'edited.zip'
    with zipfile.ZipFile(p,'w',zipfile.ZIP_DEFLATED) as z:
        for n,b in members.items():z.writestr(n,b)
    return p


def test_inner_hash_guard_even_with_replaced_outer_digest(artifact,tmp_path,monkeypatch):
    p=altered(artifact,tmp_path,lambda m:m.__setitem__('first/verification.json',b'{}'))
    monkeypatch.setattr(build,'ARCHIVE_SHA256',build.digest(p.read_bytes()))
    with pytest.raises(ValueError,match='member changed'):build.load_evidence(p)


def test_missing_member_fails_closed(artifact,tmp_path,monkeypatch):
    p=altered(artifact,tmp_path,lambda m:m.pop('repeat/verification.json'))
    monkeypatch.setattr(build,'ARCHIVE_SHA256',build.digest(p.read_bytes()))
    with pytest.raises(ValueError,match='manifest'):build.load_evidence(p)


def test_unknown_archive_member_rejected(artifact,tmp_path,monkeypatch):
    p=altered(artifact,tmp_path,lambda m:m.__setitem__('extra.json',b'{}'))
    monkeypatch.setattr(build,'ARCHIVE_SHA256',build.digest(p.read_bytes()))
    with pytest.raises(ValueError,match='manifest'):build.load_evidence(p)


def test_safe_json_cannot_terminate_data_script():
    data={'x':'</script><script>alert(1)</script>&'}
    encoded=build.safe_json(data)
    assert '<' not in encoded and '&' not in encoded
    assert json.loads(encoded)==data


def test_rebuild_is_byte_identical_and_payload_exact(artifact,data,tmp_path):
    first,second=tmp_path/'first.html',tmp_path/'second.html'
    a=build.build(artifact,first);b=build.build(artifact,second)
    assert a==b and first.read_bytes()==second.read_bytes()
    s=first.read_text()
    encoded=re.search(r'<script id="corridor-data" type="application/json">(.*?)</script>',s,re.S).group(1)
    assert json.loads(encoded)==data
    assert 'DecompressionStream' not in s and 'fetch(' not in s
    assert s.count('@@')==0


def test_fallback_has_all_three_trajectories(data):
    svg=build.fallback(data)
    assert '<svg' in svg and 'role="img"' in svg
    for color in ['#6ba9ff','#70d5ed','#61e5b3']:assert color in svg

# Publishing preparation may stage immutable objects, never mutate a branch.
import io
import sys
sys.path.insert(0,str(Path(__file__).parents[1]))
import stage_blob


def test_staging_uses_only_blob_endpoint(tmp_path):
    p=tmp_path/'example.html';p.write_bytes(b'<html>tested</html>')
    calls=[]
    def fake(req,timeout):
        calls.append(req.full_url)
        assert req.method=='POST' and req.full_url.endswith('/git/blobs')
        assert '/refs' not in req.full_url and '/commits' not in req.full_url
        return io.BytesIO(json.dumps({'sha':stage_blob.git_blob_sha(p.read_bytes())}).encode())
    result=stage_blob.stage([('docs/corridor.html',p)],'a'*40,'mock-token',opener=fake)
    assert len(calls)==1
    assert result['ref_modified'] is result['commit_created'] is result['site_deployed'] is False
    assert 'mock-token' not in json.dumps(result)


@pytest.mark.parametrize('sha',['', 'main', 'a'*39,'A'*40])
def test_staging_requires_exact_source_sha(tmp_path,sha):
    with pytest.raises(ValueError):stage_blob.stage([],sha,'test-token')


def test_staging_checks_returned_git_object(tmp_path):
    p=tmp_path/'test';p.write_bytes(b'example')
    with pytest.raises(ValueError,match='differs'):
        stage_blob.stage([('docs/corridor.html',p)],'a'*40,'mock-token',
                         opener=lambda *a,**kw:io.BytesIO(b'{"sha":"wrong"}'))


def test_staging_refuses_unverified_html(artifact,tmp_path):
    h=tmp_path/'page.html';h.write_text('unverified')
    r=tmp_path/'browser.json';r.write_text(json.dumps(dict(recordings=4,layout_cases=8,
        sample_checks=40,csv_rows=1569,script_disabled_fallback=True,restricted_iframe=True,
        mobile_no_horizontal_overflow=True,page_errors=[],console_errors=[],network_requests=[],
        html_sha256='incorrect')))
    with pytest.raises(ValueError,match='tested file'):
        stage_blob.validate_outputs(h,artifact,r,tmp_path/'missing.xml')
