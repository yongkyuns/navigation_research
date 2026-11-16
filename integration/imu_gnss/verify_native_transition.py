"""Qualify pinned native transition instrumentation against unmodified controls.

No OSM association or claimed road accuracy: the only injected map factor is a
single synthetic Gaussian lateral observation used to exercise cross-covariance
through subsequent real recorded IMU/GNSS events. The no-map control must stay
bit-identical to the existing native library for the full recording. The ABI
allocation-size metadata is reported separately because tracing adds storage.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

from datasets import load_inputs, events
from native import Library
from native_schmidt import NativeSchmidt, NativeTransition, compose_joint, covariance_psd


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def compare_navigation_snapshots(before, traced):
    """Compare every ABI field except slot 7: sizeof(Bridge), not navigation.

    Slot 7 necessarily changes when derivative-tracing storage is added. All
    other 357 slots, including health, timing, event flags, nominal state and
    the complete 18x18 covariance, must match bit for bit (no tolerance).
    Return allocation sizes for separate provenance and constancy checks.
    """
    a = np.asarray(before.raw)
    b = np.asarray(traced.raw)
    for value in (a, b):
        if value.shape != (358,) or value.dtype != np.float64:
            raise AssertionError('Unexpected native snapshot ABI')
        size = value[7]
        if not np.isfinite(size) or size <= 0 or not size.is_integer():
            raise AssertionError('Invalid native allocation-size metadata')
    if a[:7].tobytes() != b[:7].tobytes() or a[8:].tobytes() != b[8:].tobytes():
        raise AssertionError('Instrumented native navigation snapshot differs')
    return int(a[7]), int(b[7])


def verify(control_path,trace_path,dataset,output,*,max_events=None):
    imu,gnss,source=load_inputs(dataset)
    raw=Library(control_path).create()
    traced=Library(trace_path).create()
    corrected=NativeSchmidt(Library(trace_path).create(),
        'synthetic-map-patch-fixture',np.diag([4.,4.]))
    transition=NativeTransition(traced)
    outcomes=dict(schema=2,source='actual pinned native runtime events',
        dataset=source['id'],inputs=source['input_sha256'],
        default_library_sha256=sha(control_path),trace_library_sha256=sha(trace_path),
        no_map_navigation_bit_identical=True,valid_operator_events=0,
        compared_snapshot_slots=357,excluded_metadata_slot=7,
        control_allocation_bytes=None,traced_allocation_bytes=None,
        invalid_operator_events=0,total_operator_count=0,
        nontrivial_imu_operator_events=0,nontrivial_gnss_operator_events=0,
        map_update_applied=False,correlated_arm_imu=0,correlated_arm_gnss=0,
        native_reseeds_after_map=0,first_correlated_update_s=None,
        note='One controlled Gaussian lateral factor; no real-road association or accuracy claim')
    first_ready=False
    count=0
    try:
        for kind,row in events(imu,gnss):
            getattr(raw,kind)(row)
            getattr(traced,kind)(row)
            t=transition.take()
            if t.valid:
                outcomes['valid_operator_events']+=1
            else:
                outcomes['invalid_operator_events']+=1
            outcomes['total_operator_count']+=t.operators
            if t.operators>1:
                outcomes[f'nontrivial_{kind}_operator_events']+=1
            before=raw.snapshot();control=traced.snapshot()
            try:
                sizes = compare_navigation_snapshots(before, control)
            except AssertionError as exc:
                raise AssertionError(f'Native snapshot check at {kind} event {count}: {exc}') from exc
            if count == 0:
                outcomes['control_allocation_bytes'], outcomes['traced_allocation_bytes'] = sizes
            elif sizes != (outcomes['control_allocation_bytes'], outcomes['traced_allocation_bytes']):
                raise AssertionError('Native allocation metadata changed during replay')
            if not first_ready and before.ready and before.usable:
                first_ready=True
            try:
                corrected.feed(kind,row)
            except RuntimeError as exc:
                outcomes['native_reseeds_after_map']+=1
                if outcomes['map_update_applied']:
                    # A true native reseed must fail closed; no silent cross
                    # covariance reinitialization is permitted.
                    outcomes['fail_closed_reason']=str(exc)
                    break
                raise
            if first_ready and not outcomes['map_update_applied']:
                h=np.zeros(18);h[6]=1.
                j=np.array([-1.,0.])
                result=corrected.factor(patch_id='synthetic-map-patch-fixture',
                    cell='fixture:first-eligible-time',residual_m=1.,
                    h=h,j=j,local_sigma_m=.5)
                if result.applied:
                    outcomes['map_update_applied']=True
                    outcomes['first_correlated_update_s']=float(row[0])
            if outcomes['map_update_applied']:
                covariance_psd(compose_joint(corrected.nav.snapshot().P,
                    corrected.C,corrected.U),atol=2e-5)
            count+=1
            if max_events is not None and count>=max_events:break
        outcomes['event_count']=count
        outcomes['correlated_arm_imu']=corrected.imu_events
        outcomes['correlated_arm_gnss']=corrected.gnss_events
        outcomes['complete_input_replay']=count==(len(imu)+len(gnss))
        if not outcomes['map_update_applied']:
            raise AssertionError('Native filter never reached a usable initialized state')
        if outcomes['native_reseeds_after_map']:
            raise AssertionError('Native state reset after map update; research trace correctly refused stale C')
        if outcomes['total_operator_count']==0:
            raise AssertionError('No actual native transformations were recorded')
        Path(output).parent.mkdir(parents=True,exist_ok=True)
        Path(output).write_text(json.dumps(outcomes,indent=2)+'\n')
        return outcomes
    finally:
        raw.close();traced.close();corrected.close()


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--control-library',type=Path,required=True)
    ap.add_argument('--traced-library',type=Path,required=True)
    ap.add_argument('--dataset',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--max-events',type=int)
    a=ap.parse_args()
    print(json.dumps(verify(a.control_library,a.traced_library,a.dataset,a.output,
                            max_events=a.max_events),indent=2))
