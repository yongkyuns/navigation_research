"""Real-browser checks of exact positions, all recordings, controls and fallback.

Use --browser-executable for a preinstalled system Chromium, or Playwright's
managed Chromium by default. No external requests are allowed during the check.
"""
from __future__ import annotations
import argparse
import csv
import io
import hashlib
import json
from pathlib import Path
from playwright.sync_api import sync_playwright


def check(html,output,executable=None):
    source=Path(html).read_text();output=Path(output);output.mkdir(parents=True,exist_ok=True)
    report={'html_sha256':hashlib.sha256(Path(html).read_bytes()).hexdigest(),'recordings':0,'layout_cases':0,'sample_checks':0,'csv_rows':0,
            'page_errors':[],'console_errors':[],'network_requests':[],
            'script_disabled_fallback':False,'restricted_iframe':False}
    with sync_playwright() as p:
        kwargs=dict(headless=True,args=['--no-sandbox'])
        if executable:kwargs['executable_path']=executable
        browser=p.chromium.launch(**kwargs)
        report['browser_version']=browser.version
        page=browser.new_page(viewport={'width':1440,'height':1050},device_scale_factor=1)
        page.on('pageerror',lambda e:report['page_errors'].append(str(e)))
        page.on('console',lambda m:report['console_errors'].append(m.text) if m.type=='error' else None)
        page.on('request',lambda r:report['network_requests'].append(r.url))
        page.route('**/*',lambda route:route.abort())
        # Embedded previews may have no decompression or external network APIs.
        page.evaluate('window.DecompressionStream=undefined')
        page.set_content(source,wait_until='load')
        page.wait_for_function('Boolean(window.__corridorTest)')
        assert page.locator('#app').is_visible() and page.locator('#fallback').is_hidden()
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        data=page.evaluate('window.__corridorTest.data')
        # All numerical output fields from all samples survive the browser parser.
        encoded=page.locator('#corridor-data').text_content()
        assert json.loads(encoded)==data
        for di,d in enumerate(data['datasets']):
            report['recordings']+=1
            ready=next(i for i,r in enumerate(d['rows']) if r['outputs']['correlated']['ready'])
            applied=next(i for i,n in enumerate(d['diagnostics']) if n['correlated']['applied'])
            gap=next(i for i,r in enumerate(d['rows']) if r['original_edge'] is None and r['outputs']['correlated']['map_updates']>0)
            for mode in ['overlay','split']:
                page.evaluate('(args)=>{__corridorTest.set(args[0],0);__corridorTest.mode(args[1]);}',[di,mode])
                report['layout_cases']+=1
                for i in sorted(set([0,ready,applied,gap,len(d['rows'])-1])):
                    t=d['rows'][i]['elapsed_s']
                    page.evaluate('(args)=>__corridorTest.set(...args)',[di,t])
                    live=page.evaluate('__corridorTest.get()')
                    assert live['row']==d['rows'][i] and live['note']==d['diagnostics'][i]
                    assert live['index']==i and live['time']==t
                    report['sample_checks']+=1
                    if i==gap:
                        assert 'Both corrected outputs continue' in page.locator('#outputState').text_content()
                page.locator('#fit').click();page.locator('#focus').click()
            text=page.evaluate('__corridorTest.csv()');rows=list(csv.DictReader(io.StringIO(text)))
            assert len(rows)==len(d['rows'])
            for r,original,note in zip(rows,d['rows'],d['diagnostics']):
                assert float(r['elapsed_s'])==original['elapsed_s']
                for prefix,model in [('raw','disabled'),('independent','independent'),('correlated','correlated')]:
                    point=original['outputs'][model]['position']
                    if point is None:assert r[prefix+'_e_m']==r[prefix+'_n_m']==''
                    else:assert [float(r[prefix+'_e_m']),float(r[prefix+'_n_m'])]==point
                assert (r['correlated_update']=='true')==note['correlated']['applied']
            report['csv_rows']+=len(rows)
            # A time between exported epochs must never reveal a future sample.
            mid=(d['rows'][applied]['elapsed_s']+d['rows'][applied+1]['elapsed_s'])/2
            page.evaluate('t=>__corridorTest.seek(t)',mid)
            assert page.evaluate('__corridorTest.get().index')==applied
        page.evaluate('__corridorTest.set(0,200);__corridorTest.mode("overlay")')
        for key in ['raw','independent','correlated','gnss','roads','selected','covariance','full','follow']:
            page.locator('#'+key).uncheck();page.locator('#'+key).check()
        page.locator('#follow').uncheck()
        page.locator('#prev').click();i=page.evaluate('__corridorTest.get().index');page.locator('#next').click();assert page.evaluate('__corridorTest.get().index')==i+1
        page.locator('#gap').click();gap_state=page.evaluate('__corridorTest.get()');assert gap_state['row']['original_edge'] is None and gap_state['row']['outputs']['correlated']['ready']
        page.locator('#max').click();mx=page.evaluate('__corridorTest.get()');assert mx['note']['correlated']['native_position_step_m']==max(n['correlated']['native_position_step_m'] for n in data['datasets'][0]['diagnostics'])
        # Inspect a real supplied road at the nearest point to the current pose.
        click=page.evaluate('''()=>{const q=__corridorTest.get(),r=q.row,road=__corridorTest.data.roads.find(e=>e[0]===r.original_edge),p=r.raw_position;let best=null;for(let i=1;i<road[3].length;i++){const a=road[3][i-1],b=road[3][i],dx=b[0]-a[0],dy=b[1]-a[1],u=Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)));const point=[a[0]+u*dx,a[1]+u*dy],dist=Math.hypot(point[0]-p[0],point[1]-p[1]);if(!best||dist<best.dist)best={point,dist};}const c=document.getElementById('map0').getBoundingClientRect();return [c.x+c.width/2+(best.point[0]-q.view.x)*q.view.s,c.y+c.height/2-(best.point[1]-q.view.y)*q.view.s];}''')
        page.mouse.click(*click);page.wait_for_timeout(40)
        assert 'Inspected ' in page.locator('#inspected').text_content()
        page.locator('#difference').click();delta=page.evaluate('__corridorTest.get()');assert delta['index']==168
        page.locator('#max').click()
        page.locator('#play').click();page.wait_for_timeout(300);page.locator('#play').click();assert page.evaluate('__corridorTest.get().time')>mx['time']
        # Zoom and drag affect only the camera, not displayed numerical outputs.
        row=page.evaluate('__corridorTest.get().row');before=page.evaluate('__corridorTest.get().view')
        canvas=page.locator('#map0');box=canvas.bounding_box();x=box['x']+box['width']/2;y=box['y']+box['height']/2
        page.mouse.move(x,y);page.mouse.wheel(0,-180);page.wait_for_timeout(70)
        assert page.evaluate('__corridorTest.get().view.s')>before['s']
        page.mouse.down();page.mouse.move(x+30,y+20,steps=4);page.mouse.up();page.wait_for_timeout(70)
        assert page.evaluate('__corridorTest.get().row')==row
        # Real export-button download, not just the CSV helper.
        with page.expect_download() as download:page.locator('#export').click()
        dest=output/'browser-export.csv';download.value.save_as(dest)
        assert dest.read_text()==page.evaluate('__corridorTest.csv()')
        page.locator('#difference').click();page.locator('#covariance').uncheck();page.locator('#follow').uncheck()
        page.screenshot(path=str(output/'desktop.png'),full_page=True)
        page.locator('#split').click();page.locator('#fit').click();page.screenshot(path=str(output/'split.png'),full_page=True)
        page.set_viewport_size({'width':390,'height':844});page.locator('#overlay').click();page.locator('#max').click();page.wait_for_timeout(100)
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.screenshot(path=str(output/'mobile.png'),full_page=True)
        report['mobile_no_horizontal_overflow']=True
        # Same generated page as sandboxed srcdoc, no same-origin or network permission.
        framepage=browser.new_page(viewport={'width':1100,'height':850})
        framepage.on('pageerror',lambda e:report['page_errors'].append(str(e)))
        framepage.on('request',lambda r:report['network_requests'].append(r.url))
        framepage.on('console',lambda m:report['console_errors'].append(m.text) if m.type=='error' else None)
        framepage.route('**/*',lambda r:r.abort())
        framepage.set_content('<iframe id="preview" sandbox="allow-scripts" style="width:100%;height:800px;border:0"></iframe>')
        framepage.locator('#preview').evaluate('(e,s)=>{e.srcdoc=s}',source)
        frame=framepage.frame_locator('#preview');frame.locator('#dataset').wait_for()
        assert frame.locator('#app').is_visible()
        frame.locator('#gap').click();assert 'Both corrected outputs continue' in frame.locator('#outputState').text_content()
        report['restricted_iframe']=True
        fallback=browser.new_context(java_script_enabled=False).new_page()
        fallback.set_content(source,wait_until='load')
        assert fallback.locator('#fallback').is_visible() and fallback.locator('#app').is_hidden()
        assert fallback.locator('#fallback svg polyline').count()>3
        report['script_disabled_fallback']=True
        assert not report['page_errors'] and not report['console_errors'] and not report['network_requests'],report
        browser.close()
    (output/'browser-checks.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--html',required=True,type=Path);ap.add_argument('--output',required=True,type=Path);ap.add_argument('--browser-executable')
    a=ap.parse_args();print(json.dumps(check(a.html,a.output,a.browser_executable),indent=2))
