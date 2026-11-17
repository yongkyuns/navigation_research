"""Offline negative controls for the public-site verifier, not public HTTP evidence."""
import json
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import verify_publication as v

BODY = b'<html>exact reviewed bytes</html>'
URL = v.PAGES['corridor.html']


def response(**changes):
    kwargs = dict(status=200, url=URL, headers={'Content-Type': 'text/html; charset=utf-8'},
                  expected=BODY, expected_url=URL)
    kwargs.update(changes)
    return v.validate_response(BODY, **kwargs)


def test_exact_public_response_is_verified_without_modification():
    result = response()
    assert result['byte_identical'] and result['sha256'] == v.sha256(BODY)
    assert result['bytes'] == len(BODY) and result['url'] == URL


@pytest.mark.parametrize('status', [204, 206, 301, 302, 404, 500])
def test_status_is_not_silently_accepted(status):
    with pytest.raises(ValueError, match='status'):
        response(status=status)


@pytest.mark.parametrize('url', ['http://yongkyuns.github.io/navigation_research/corridor.html',
    URL + '?other=1', v.SITE, 'https://example.org/corridor.html'])
def test_redirect_or_other_origin_is_not_a_pass(url):
    with pytest.raises(ValueError, match='URL'):
        response(url=url)


@pytest.mark.parametrize('expected', [BODY + b' ', BODY[:-1], BODY.replace(b'exact', b'stale')])
def test_stale_truncated_or_extra_content_fails(expected):
    with pytest.raises(ValueError, match='mismatch'):
        response(expected=expected)


@pytest.mark.parametrize('headers', [{}, {'Content-Type': 'application/octet-stream'},
    {'Content-Type': 'text/html', 'Content-Encoding': 'gzip'},
    {'Content-Type': 'text/html', 'Content-Length': '999'},
    {'Content-Type': 'text/html', 'Content-Length': 'not-a-number'}])
def test_invalid_transport_metadata_fails(headers):
    with pytest.raises(ValueError):
        response(headers=headers)


def test_identity_encoding_and_correct_length_pass():
    response(headers={'CONTENT-TYPE': 'TEXT/HTML; charset=UTF-8',
                      'Content-Encoding': 'identity', 'Content-Length': str(len(BODY))})


def test_redirect_handler_declines_redirects():
    assert v.NoRedirect().redirect_request(None, None, 302, '', {}, 'https://example.org') is None


def test_fetch_rejects_unlisted_url_before_network():
    with pytest.raises(ValueError, match='canonical'):
        v.fetch_page('https://example.org/', BODY)


def test_embedded_data_parser_reads_only_target_script():
    html = b'<script>throw 1</script><script type="application/json" id="corridor-data">{"x":[1,2]}</script>'
    assert v.embedded_data(html) == {'x': [1, 2]}


@pytest.mark.parametrize('html', [b'<html/>',
    b'<script id="corridor-data">{}</script><script id="corridor-data">{}</script>',
    b'<script id="corridor-data">garbled</script>'])
def test_missing_duplicate_or_garbled_embedded_data_fails(html):
    with pytest.raises(ValueError):
        v.embedded_data(html)


def setup_docs(tmp_path):
    docs = tmp_path / 'docs'; docs.mkdir()
    for name in v.PAGES:
        (docs / name).write_bytes(BODY)
    return docs


def test_failure_report_never_claims_complete(tmp_path, monkeypatch):
    docs = setup_docs(tmp_path)
    def failure(*args):
        raise OSError('recorded network failure')
    monkeypatch.setattr(v, 'fetch_page', failure)
    out = tmp_path / 'result'
    with pytest.raises(OSError):
        v.verify(docs, 'a' * 40, out, live_browser=True)
    result = json.loads((out / 'publication-verification.json').read_text())
    assert result['complete'] is False and result['pages'] == {}
    assert result['scope'] == 'public-http-and-browser'
    assert 'recorded network failure' in result['error']


def test_browser_failure_preserves_http_evidence_but_not_overall_pass(tmp_path, monkeypatch):
    docs = setup_docs(tmp_path)
    def fetch(url, expected):
        return expected, {'url': url, 'sha256': v.sha256(expected), 'byte_identical': True}
    def browser_failure(body, output, report, executable):
        report.update(complete=False, page_errors=['script blocked'])
        raise AssertionError('browser failed')
    monkeypatch.setattr(v, 'fetch_page', fetch)
    monkeypatch.setattr(v, 'browser_smoke', browser_failure)
    out = tmp_path / 'result'
    with pytest.raises(AssertionError):
        v.verify(docs, 'b' * 40, out, live_browser=True)
    result = json.loads((out / 'publication-verification.json').read_text())
    assert not result['complete'] and not result['browser']['complete']
    assert len(result['pages']) == 2


def test_http_only_does_not_claim_browser_execution(tmp_path, monkeypatch):
    docs = setup_docs(tmp_path)
    monkeypatch.setattr(v, 'fetch_page', lambda url, data:
                        (data, {'url': url, 'sha256': v.sha256(data), 'byte_identical': True}))
    result = v.verify(docs, 'c' * 40, tmp_path / 'result')
    assert result['complete'] and result['scope'] == 'public-http-only'
    assert result['browser'] == {} and len(result['pages']) == 2


@pytest.mark.parametrize('revision', ['main', 'c76a409', '', 'z' * 40])
def test_expected_revision_cannot_be_a_moving_branch(tmp_path, revision):
    with pytest.raises(ValueError, match='immutable'):
        v.verify(tmp_path, revision, tmp_path / 'result')


def test_workflow_has_read_only_repository_permissions():
    workflow = Path(__file__).resolve().parents[3] / '.github/workflows/verify-playback-publication.yml'
    text = workflow.read_text()
    assert 'contents: read' in text and 'contents: write' not in text
    assert 'pages: write' not in text and 'id-token: write' not in text
    assert 'persist-credentials: false' in text
    assert '--browser' in text
    assert 'pull_request_target:' not in text
    assert 'workflow_run:' in text and 'pages build and deployment' in text
