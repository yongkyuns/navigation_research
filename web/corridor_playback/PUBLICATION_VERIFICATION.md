# Public playback verification

A green Pages deployment and a byte-verified deployment artifact do not prove a
public HTTP client received that content or that the page's scripts initialized
under its actual hosting headers. This read-only check closes that separate gate.
It never changes pages, navigation code, estimator outputs, branch refs or site
configuration, and requires no token for either public HTTP or browser requests.

`verify_publication.py` compares both canonical public pages (`/navigation_research/`
and `/navigation_research/corridor.html`) to `docs/index.html` and
`docs/corridor.html` from an explicitly identified immutable Git revision. It
requires HTTP 200, unchanged URL, HTML content type and exact response bytes.
Redirects, stale/truncated/extra content and unexpected encoding fail closed.
Fetched bytes and metadata are retained; a failed check is not labeled complete.

With `--browser`, Chromium actually navigates to the public corridor URL; it does
not substitute local HTML with `set_content` or route fulfillment. Its response
body must also match. The smoke check compares parsed retained data, all four
recordings in both layouts at initialization/association-gap/final epochs (24
sample/diagnostic comparisons), playback/pause and mobile overflow. Screenshots,
script/console errors and requests are retained. Unexpected network dependencies
fail. The existing more extensive offline viewer test suite remains unchanged.

The workflow has only `contents: read`; it has no deployment or repository-write
permission. On a PR it tests the **already published base revision**, not the PR's
unpublished docs. After a successful main-branch Pages workflow, it tests that
exact deployment's source SHA. A manual run tests its selected revision. Checkout
SHAs are verified and reported; no older deployment is silently substituted to
obtain a pass. An unrelated deployment during the test may cause a content mismatch
and requires investigation, not a relaxed hash comparison.

Run against the corresponding checked-out source:

```sh
python -m pip install pytest==9.0.2 playwright==1.57.0
python -m playwright install chromium
python -m pytest -q web/corridor_playback/publication_tests
python web/corridor_playback/verify_publication.py \
  --expected-docs docs --expected-revision "$(git rev-parse HEAD)" \
  --output runs/publication --browser
```

`--browser-executable` accepts a system Chromium binary for local testing. Omitting
`--browser` is an explicitly labeled HTTP-only check, not live-browser evidence.
Tests with mocked responses are negative controls; only a completed live CI report
establishes public availability. No surveyed accuracy or new native run is implied.
