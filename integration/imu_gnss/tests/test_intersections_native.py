"""Real Rust state/chronology tests on recorded inputs and synthetic junctions.

The fixture roads exercise the ABI and policy, not field route accuracy. No
reference stream is used to initialize or update the native runtime.
"""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from native import Frame
from road_bank import RoadBank, Branch
from routefusion.tiles import Edge
from datasets import events
from test_integration import library, recording, initialized
from test_intersections import Roads


def native_bank(library, recording):
    nav = initialized(library, recording)
    snapshot = nav.snapshot()
    frame = Frame(snapshot.anchor)
    p, _, velocity = snapshot.pose(frame)
    tangent = velocity / np.linalg.norm(velocity)
    normal = np.array([-tangent[1], tangent[0]])
    center = p + 20 * tangent
    roads = Roads([
        Edge('in', [p - 100 * tangent, center], ('straight', 'left', 'right'), 6),
        Edge('straight', [center, center + 150 * tangent], (), 6),
        Edge('left', [center, center + 150 * normal], (), 6),
        Edge('right', [center, center - 150 * normal], (), 6),
    ])
    roads.frame = frame
    bank = RoadBank(nav, roads)
    bank.branches = [Branch('in', nav.clone(), np.zeros(2), 0.)]
    return bank


def test_native_junction_branches_have_independent_full_error_states(library, recording):
    bank = native_bank(library, recording)
    before = bank.base.snapshot().raw.copy()
    out = bank.step(30.)
    assert len(bank.branches) == 3 and not out['latched']
    assert {b.maneuver.path[-1] for b in bank.branches} == {'left', 'right', 'straight'}
    other = bank.branches[1].nav.snapshot().raw.copy()
    dx = np.zeros(18)
    dx[6] = 1.
    bank.branches[0].nav.inject(dx, bank.branches[0].nav.snapshot().P)
    np.testing.assert_array_equal(bank.branches[1].nav.snapshot().raw, other)
    np.testing.assert_array_equal(bank.base.snapshot().raw, before)
    bank.reset()
    bank.base.close()


def test_native_checkpoint_replay_preserves_all_original_updates(library, recording):
    bank = native_bank(library, recording)
    bank.step(30.)
    assert bank.window.nav is not None
    control = bank.base.clone()
    imu, gnss, _ = recording
    consumed = {'imu': 0, 'gnss': 0}
    for kind, row in events(imu, gnss):
        if row[0] <= 30:
            continue
        if row[0] > 31:
            break
        getattr(bank, kind)(row)
        getattr(control, kind)(row)
        consumed[kind] += 1
    bank.step(31.)
    assert consumed['imu'] > 40 and consumed['gnss'] > 0
    before = bank.base.snapshot().raw.copy()
    np.testing.assert_array_equal(before, control.snapshot().raw)
    bank.close_branches()  # fault injection, not feedback from the control
    assert bank._recover(bank.base.snapshot())
    np.testing.assert_array_equal(bank.base.snapshot().raw, before)
    assert bank.recoveries == 1 and not bank.window.events
    # A second attempt cannot assimilate the same window again.
    assert not bank._recover(bank.base.snapshot())
    bank.reset()
    bank.base.close()
    control.close()
