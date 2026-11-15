"""Check comparison integrity; CI success is NOT universal route-latching success."""
from pathlib import Path
import argparse
import json
import xml.etree.ElementTree as ET
import numpy as np


def qualify(root: Path, tests: Path):
    doc=json.loads((root/'summary.json').read_text());rows=doc['results'];cfg=doc['config']
    assert len(rows)==51 and len({r['id'] for r in rows})==51
    assert all(r['fallback_bit_identical'] and r['input_arrays_unchanged'] for r in rows)
    assert all(not r['bias_model_enabled'] and r['native_error_states']==18 for r in rows)
    assert all(r['minimum_known_covariance_eigenvalue']>=-1e-6 for r in rows)
    for route in cfg['routes']:
        cases=[r for r in rows if r['route']['name']==route['name']]
        assert len(cases)==17
        assert len({json.dumps(r['input_provenance']['input_sha256'],sort_keys=True) for r in cases})==1
    suites=list(ET.parse(tests).getroot().iter('testsuite'))
    assert suites and not any(int(s.get(k,0)) for s in suites for k in ('errors','failures','skipped'))
    count=sum(int(s.get('tests',0)) for s in suites)
    assert count>=14
    groups=[]
    for prior in cfg['prior_modes']:
        for feed in ['original_gnss','withhold_first_10s']:
            for offset in [10,20]:
                cases=[r for r in rows if r['prior']==prior and r['gnss_mode']==feed and
                       r['target_cross_m'] is not None and abs(r['target_cross_m'])==offset]
                assert len(cases)==6
                group={'initial_cross_magnitude_m':offset,'prior':prior,'gnss':feed,'cases':len(cases)}
                for key in ['before','known_route','automatic','automatic_accepted_native_latch']:
                    times=[r[key]['first_5s_hold_entry_s'] for r in cases]
                    valid=[t for t in times if t is not None]
                    group[key]={'entries_before_10s':sum(t is not None and t<10 for t in times),
                                'entry_times_s':times,'range_s':[min(valid),max(valid)] if valid else None,
                                'mean_first10_inside_fraction':float(np.mean([r[key]['first_10s_inside_fraction'] for r in cases]))}
                groups.append(group)
    result={'experiment_integrity_passed':True,'cases':len(rows),'route_fixtures':3,
            'new_tests':count,'gnss_bias_model_enabled':False,'paired_fallback_bit_identical':True,
            'default_navigation_modified':False,'surveyed_accuracy_claim':False,
            'requirement_status':'See observed outcomes; execution integrity does not establish universal latching.',
            'groups':groups,'source_sha256':doc['source_sha256'],'config_sha256':doc['config_sha256'],
            'library_sha256':doc['library_sha256'],'map_response_sha256':doc['map_response_sha256']}
    (root/'qualification.json').write_text(json.dumps(result,indent=2)+'\n')
    lines=['# Observed route-latching outcomes','',
           'All distances are relative to supplied OSM routes, not surveyed truth. Receiver bias modelling is disabled. Initialization uncertainty diagnostics are separate from native-covariance results.','',
           '| Prior | Fixes | Initial cross-track | Before entry <10 s | Supplied-route entry <10 s | Accepted automatic route entry <10 s |',
           '|---|---|---:|---:|---:|---:|']
    for g in groups:
        values=[str(g[k]['entries_before_10s'])+'/6' for k in ['before','known_route','automatic_accepted_native_latch']]
        lines.append('| '+g['prior']+' | '+g['gnss']+' | '+str(g['initial_cross_magnitude_m'])+' m | '+' | '.join(values)+' |')
    lines+=['','Entry must be followed by five sampled seconds inside. The hold may extend past GNSS resumption at 10 s. Automatic entry additionally requires an accepted native branch on the supplied itinerary; a good fallback does not count. Full time-weighted residence and retention are recorded per case.','',
            'The first sample is before the map update; a second sample at the same time is the actual corrected native state. It owns the following time interval. Rejected/reused factors are checked to leave the native state unchanged.','',
            'No source filter, road width, sensor tuning, map factor, intersection rule or default publication path is changed by this experiment. See the accompanying README for interpretation and limitations.']
    (root/'OUTCOMES.md').write_text('\n'.join(lines)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results',type=Path,required=True);p.add_argument('--tests',type=Path,required=True)
    a=p.parse_args();q=qualify(a.results,a.tests)
    print(json.dumps({k:v for k,v in q.items() if k not in ('groups','source_sha256')},indent=2))
