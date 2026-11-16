"""Stage tested immutable Git blobs; NEVER create a commit or change a ref.

A maintainer subsequently attaches these exact objects to the reviewed PR and
requires new-head CI before merging. This is not a Pages deployment action.
"""
from __future__ import annotations
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import re
import urllib.request
import xml.etree.ElementTree as ET
from build import ARCHIVE_SHA256, digest

REPO='yongkyuns/navigation_research'


def git_blob_sha(content):
    return hashlib.sha1(b'blob '+str(len(content)).encode()+b'\0'+content).hexdigest()


def validate_outputs(html,archive,report,unit_xml):
    if digest(archive.read_bytes())!=ARCHIVE_SHA256:
        raise ValueError('Wrong qualified native evidence')
    audit=json.loads(report.read_text())
    if not (audit['recordings']==4 and audit['layout_cases']==8
            and audit['sample_checks']>=40 and audit['csv_rows']==1569
            and audit['script_disabled_fallback'] is True and audit['restricted_iframe'] is True
            and audit['mobile_no_horizontal_overflow'] is True
            and not audit['page_errors'] and not audit['console_errors'] and not audit['network_requests']):
        raise ValueError('Browser qualification incomplete')
    if audit.get('html_sha256')!=digest(html.read_bytes()):
        raise ValueError('HTML is not the tested file')
    suites=ET.parse(unit_xml).getroot().findall('.//testsuite')
    if not suites or sum(int(s.get('tests','0')) for s in suites)<10:
        raise ValueError('Unit qualification incomplete')
    if any(int(s.get(k,'0')) for s in suites for k in ('failures','errors','skipped')):
        raise ValueError('Unit qualification failed or skipped')
    return [('docs/corridor.html',html),('artifacts/corridor-playback-pr13-ci.zip',archive)]


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self,*args,**kwargs):return None


def stage(objects,source_sha,token,*,opener=None):
    if not re.fullmatch('[0-9a-f]{40}',source_sha) or not token:
        raise ValueError('Exact source revision and repository token required')
    if opener is None:opener=urllib.request.build_opener(NoRedirect()).open
    results=[]
    for name,path in objects:
        content=path.read_bytes()
        req=urllib.request.Request('https://api.github.com/repos/'+REPO+'/git/blobs',
            data=json.dumps(dict(content=base64.b64encode(content).decode(),encoding='base64')).encode(),
            headers={'Authorization':'Bearer '+token,'Accept':'application/vnd.github+json',
                     'Content-Type':'application/json','X-GitHub-Api-Version':'2022-11-28'},method='POST')
        with opener(req,timeout=120) as response:result=json.load(response)
        if result['sha']!=git_blob_sha(content):raise ValueError('Staged Git object differs')
        results.append(dict(path=name,blob_sha=result['sha'],sha256=digest(content),size_bytes=len(content)))
    return dict(source_sha=source_sha,objects=results,ref_modified=False,commit_created=False,site_deployed=False)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    for key in ('html','artifact','browser-report','unit-xml','output'):ap.add_argument('--'+key,type=Path,required=True)
    args=ap.parse_args()
    if os.environ.get('GITHUB_REPOSITORY')!=REPO:raise ValueError('Unexpected repository')
    files=validate_outputs(args.html,args.artifact,args.browser_report,args.unit_xml)
    result=stage(files,os.environ['SOURCE_SHA'],os.environ['GH_TOKEN'])
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
