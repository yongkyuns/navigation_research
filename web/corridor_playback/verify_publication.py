"""Read-only public HTTP and Chromium verification of already-published playback.

Compare the real HTTPS responses to files from an explicitly identified Git
revision. This does not build, publish, retry deployment, run the estimator, or
change GitHub settings. Redirects and unexpected browser dependencies fail.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import re
from urllib.request import HTTPRedirectHandler, Request, build_opener

SITE = 'https://yongkyuns.github.io/navigation_research/'
PAGES = {'index.html': SITE, 'corridor.html': SITE + 'corridor.html'}


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def validate_response(body, *, status, url, headers, expected, expected_url):
    """Validate content and transport before calling a page publicly verified."""
    if status != 200:
        raise ValueError(f'Public response status is {status}, not 200')
    if url != expected_url:
        raise ValueError('Public response URL changed')
    headers = {k.lower(): v for k, v in headers.items()}
    if headers.get('content-type', '').split(';')[0].strip().lower() != 'text/html':
        raise ValueError('Public response is not HTML')
    if headers.get('content-encoding', 'identity').strip().lower() != 'identity':
        raise ValueError('Unexpected HTTP content encoding')
    if 'content-length' in headers and int(headers['content-length']) != len(body):
        raise ValueError('HTTP content length differs from received bytes')
    if body != expected:
        raise ValueError(f'Public content mismatch: expected {sha256(expected)}, got {sha256(body)}')
    return {'url': url, 'status': status, 'bytes': len(body), 'sha256': sha256(body),
            'byte_identical': True, 'content_type': headers['content-type']}


def fetch_page(url: str, expected: bytes):
    if url not in PAGES.values():
        raise ValueError('Only the two canonical playback URLs are supported')
    req = Request(url, method='GET', headers={
        'User-Agent': 'navigation-research-publication-verifier/1',
        'Accept': 'text/html', 'Accept-Encoding': 'identity', 'Cache-Control': 'no-cache'})
    # The public request has no token, cookie, or authorization header. Reading
    # one byte beyond expected length detects truncation/extra data with a bound.
    with build_opener(NoRedirect()).open(req, timeout=30) as response:
        body = response.read(len(expected) + 1)
        result = validate_response(body, status=response.status, url=response.geturl(),
            headers=response.headers, expected=expected, expected_url=url)
    return body, result


class EmbeddedData(HTMLParser):
    def __init__(self):
        super().__init__()
        self.active = False
        self.parts = []
        self.matches = 0

    def handle_starttag(self, tag, attrs):
        if tag == 'script' and dict(attrs).get('id') == 'corridor-data':
            self.active = True
            self.matches += 1

    def handle_endtag(self, tag):
        if tag == 'script':
            self.active = False

    def handle_data(self, data):
        if self.active:
            self.parts.append(data)


def embedded_data(html: bytes):
    parser = EmbeddedData()
    parser.feed(html.decode('utf-8'))
    if parser.matches != 1:
        raise ValueError('Expected one embedded corridor dataset')
    return json.loads(''.join(parser.parts))


def browser_smoke(expected: bytes, output: Path, report: dict, executable=None):
    """Navigate to the actual public URL; never fulfill it with local HTML."""
    from playwright.sync_api import sync_playwright
    data = embedded_data(expected)
    report.update(complete=False, url=PAGES['corridor.html'], network_requests=[],
                  unexpected_requests=[], page_errors=[], console_errors=[],
                  recordings=0, layouts=0, sample_checks=0)
    with sync_playwright() as p:
        options = {'headless': True, 'args': ['--no-sandbox']}
        if executable:
            options['executable_path'] = executable
        browser = p.chromium.launch(**options)
        try:
            context = browser.new_context(service_workers='block',
                viewport={'width': 1440, 'height': 1050})
            page = context.new_page()
            page.on('pageerror', lambda e: report['page_errors'].append(str(e)))
            page.on('console', lambda m: report['console_errors'].append(m.text)
                    if m.type == 'error' else None)
            def route(request_route):
                request = request_route.request
                report['network_requests'].append(request.url)
                if (request.url == PAGES['corridor.html'] and request.method == 'GET'
                        and request.is_navigation_request() and request.frame == page.main_frame):
                    request_route.continue_()
                else:
                    report['unexpected_requests'].append(request.url)
                    request_route.abort()
            context.route('**/*', route)
            response = page.goto(PAGES['corridor.html'], wait_until='load', timeout=45000)
            if response is None or response.status != 200 or response.url != PAGES['corridor.html']:
                raise AssertionError('Public browser navigation did not return the expected page')
            # Playwright returns the decoded resource body. Unlike the separate
            # identity-encoded HTTP request, the browser may negotiate gzip/br.
            if response.body() != expected:
                raise AssertionError('Browser received different HTML from the verified revision')
            report['html_sha256'] = sha256(response.body())
            report['browser_version'] = browser.version
            page.wait_for_function('Boolean(window.__corridorTest)', timeout=20000)
            if not page.locator('#app').is_visible() or not page.locator('#fallback').is_hidden():
                raise AssertionError('Public page did not initialize its interactive viewer')
            if page.evaluate('__corridorTest.data') != data:
                raise AssertionError('Public browser parsed different retained numerical data')
            for di, recording in enumerate(data['datasets']):
                report['recordings'] += 1
                ready = next(i for i, row in enumerate(recording['rows'])
                             if row['outputs']['correlated']['ready'])
                gap = next(i for i, row in enumerate(recording['rows'])
                           if row['original_edge'] is None and row['outputs']['correlated']['map_updates'] > 0)
                for mode in ('overlay', 'split'):
                    page.evaluate('(m)=>__corridorTest.mode(m)', mode)
                    report['layouts'] += 1
                    for i in (ready, gap, len(recording['rows']) - 1):
                        row = recording['rows'][i]
                        page.evaluate('(v)=>__corridorTest.set(...v)', [di, row['elapsed_s']])
                        actual = page.evaluate('__corridorTest.get()')
                        if actual['row'] != row or actual['note'] != recording['diagnostics'][i]:
                            raise AssertionError('Public viewer changed a retained sample or diagnostic')
                        if i == gap and 'Both corrected outputs continue' not in page.locator('#outputState').inner_text():
                            raise AssertionError('Corrected output missing during association gap')
                        report['sample_checks'] += 1
            page.evaluate('__corridorTest.set(0,100);__corridorTest.mode("overlay")')
            start = page.evaluate('__corridorTest.get().time')
            page.locator('#play').click()
            page.wait_for_timeout(300)
            page.locator('#play').click()
            stopped = page.evaluate('__corridorTest.get().time')
            if stopped <= start:
                raise AssertionError('Public playback clock did not advance')
            page.wait_for_timeout(100)
            if page.evaluate('__corridorTest.get().time') != stopped:
                raise AssertionError('Public playback did not pause')
            report['playback_and_pause'] = True
            page.locator('#difference').click()
            page.screenshot(path=str(output / 'public-desktop.png'), full_page=True)
            page.set_viewport_size({'width': 390, 'height': 844})
            if not page.evaluate('document.documentElement.scrollWidth<=innerWidth'):
                raise AssertionError('Public mobile page overflows horizontally')
            report['mobile_no_horizontal_overflow'] = True
            page.screenshot(path=str(output / 'public-mobile.png'), full_page=True)
            if (report['recordings'], report['layouts'], report['sample_checks']) != (4, 8, 24):
                raise AssertionError('Incomplete live browser comparison coverage')
            if report['page_errors'] or report['console_errors'] or report['unexpected_requests']:
                raise AssertionError('Public browser errors or unexpected network dependencies')
            report['complete'] = True
        finally:
            browser.close()


def verify(expected_docs, expected_revision, output, *, live_browser=False, executable=None):
    if not re.fullmatch('[0-9a-f]{40}', expected_revision):
        raise ValueError('Expected a full immutable Git commit SHA')
    root, output = Path(expected_docs), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = dict(schema=1, checked_at=datetime.now(timezone.utc).isoformat(),
        expected_revision=expected_revision, verifier_sha256=sha256(Path(__file__).read_bytes()),
        scope='public-http-and-browser' if live_browser else 'public-http-only',
        complete=False, pages={}, browser={})
    try:
        fetched = {}
        for name, url in PAGES.items():
            expected = (root / name).read_bytes()
            if not expected:
                raise ValueError('Expected page cannot be empty')
            body, details = fetch_page(url, expected)
            fetched[name] = body
            report['pages'][name] = details
            (output / name).write_bytes(body)
        if live_browser:
            browser_smoke(fetched['corridor.html'], output, report['browser'], executable)
        report['complete'] = True
        return report
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        (output / 'publication-verification.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-docs', type=Path, required=True)
    parser.add_argument('--expected-revision', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--browser', action='store_true')
    parser.add_argument('--browser-executable')
    args = parser.parse_args()
    print(json.dumps(verify(args.expected_docs, args.expected_revision, args.output,
        live_browser=args.browser, executable=args.browser_executable), indent=2))
