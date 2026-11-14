"""Intersection policy regressions; deterministic test double is NOT field data.

The fake navigation endpoint moves only from supplied chronological samples.
Native-Rust compatibility remains covered separately in test_integration.py.
"""
from dataclasses import replace
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from native import Frame
from road_bank import RoadBank, Branch
from routefusion.tiles import Edge, TileStore
from intersections import (JunctionConfig, MissingTopology, HypothesisOverflow,
                           maneuver_paths, advance)
from recovery import ReplayWindow


class Snapshot:
    def __init__(self, nav):
        self.ready = self.usable = True
        self.anchor = np.array([0., 0., 0.])
        self.lla = self.anchor.copy()
        self.P = nav.P.copy()
        self.position = np.array([nav.p[1], nav.p[0], 0.])
        self.velocity = np.array([nav.v[1], nav.v[0], 0.])
        self.raw = np.r_[nav.p, nav.v, nav.t, nav.count, nav.P.ravel()]
        self.p, self.v = nav.p.copy(), nav.v.copy()

    def pose(self, frame):
        return self.p.copy(), self.P[np.ix_([7, 6], [7, 6])], self.v.copy()


class FakeNav:
    live = 0
    peak = 0

    def __init__(self, p=(-40., 0.), v=(10., 0.)):
        self.p, self.v = np.array(p, dtype=float), np.array(v, dtype=float)
        self.P = np.eye(18)
        self.t = -1.
        self.count = 0
        self.closed = False
        FakeNav.live += 1
        FakeNav.peak = max(FakeNav.peak, FakeNav.live)

    def clone(self):
        result = FakeNav(self.p, self.v)
        result.P = self.P.copy()
        result.t, result.count = self.t, self.count
        return result

    def close(self):
        if not self.closed:
            FakeNav.live -= 1
            self.closed = True

    def snapshot(self):
        assert not self.closed
        return Snapshot(self)

    def imu(self, row):
        assert not self.closed and row[0] > self.t
        self.t = row[0]
        self.p, self.v = np.array(row[1:3]), np.array(row[3:5])
        self.count += 1

    def gnss(self, row):
        # Count exactly-once delivery; policy tests deliberately use no GNSS
        # position likelihood. Full native replay covers actual PVT updates.
        assert not self.closed
        self.count += 1

    def inject(self, dx, P):
        self.p += dx[[7, 6]]
        self.P = P.copy()


class Roads:
    def __init__(self, edges):
        self.frame = Frame([0., 0., 0.])
        self.store = TileStore(version='intersection-fixture-v1')
        for edge in edges:
            self.store.add(edge)

    def covers(self, lla):
        return True


def crossroad(*, same_group=False):
    return Roads([
        Edge('in', [[-100, 0], [0, 0]], ('straight', 'left', 'right'), 4., group='incoming'),
        Edge('straight', [[0, 0], [150, 0]], (), 4., group='same' if same_group else 'straight'),
        Edge('left', [[0, 0], [0, 150]], (), 4., group='same' if same_group else 'left'),
        Edge('right', [[0, 0], [0, -150]], (), 4., group='same' if same_group else 'right'),
    ])


def bank_at(roads=None, cap=24, config=None):
    base = FakeNav()
    bank = RoadBank(base, roads or crossroad(), cap, config)
    bank.branches = [Branch('in', base.clone(), np.zeros(2), 0.)]
    return bank


def feed(bank, t, p, v):
    bank.imu(np.array([t, *p, *v, 0., 0.]))
    return bank.step(t)


def close(bank):
    bank.reset()
    bank.base.close()


@pytest.mark.parametrize('target,delta', [('straight', (1, 0)), ('left', (0, 1)), ('right', (0, -1))])
def test_exit_survives_approach_and_resolves_after_crossing(target, delta):
    bank = bank_at()
    out = feed(bank, 0., (-20, 0), (10, 0))
    assert {b.maneuver.path[-1] for b in bank.branches} == {'left', 'right', 'straight'}
    assert not out['latched']
    # The shared incoming-road update may reduce cross-track covariance, but
    # it must not favor or bend toward any future turn.
    for b in bank.branches:
        np.testing.assert_array_equal(b.nav.snapshot().position, bank.base.snapshot().position)
        np.testing.assert_array_equal(b.nav.snapshot().raw, bank.branches[0].nav.snapshot().raw)
    out = feed(bank, 1., (-5, 0), (10, 0))
    assert len(bank.branches) == 3 and out['status'] == 'JUNCTION_AMBIGUOUS'
    for t, distance in enumerate((5., 20., 30., 40., 50.), start=2):
        out = feed(bank, float(t), np.array(delta) * distance, np.array(delta) * 10.)
        if distance <= 20:
            assert not out['latched']
    assert out['latched'] and out['edge'] == target
    assert out['position'].tolist() == (np.array(delta) * 50.).tolist()
    close(bank)


def test_stationary_junction_retains_modes_without_covariance_collapse():
    bank = bank_at()
    feed(bank, 0., (-20, 0), (10, 0))
    out = feed(bank, 1., (0, 0), (0, 0))
    before = [b.nav.snapshot().P.copy() for b in bank.branches]
    for t in range(2, 15):
        out = feed(bank, float(t), (0, 0), (0, 0))
        assert not out['latched'] and len(bank.branches) == 3
    for b, P in zip(bank.branches, before):
        np.testing.assert_array_equal(b.nav.snapshot().P, P)
    close(bank)


def test_outgoing_heading_is_not_required_before_a_right_angle_turn():
    roads = crossroad()
    m = next(m for m in maneuver_paths(roads.store, 'in', JunctionConfig()) if m.path[-1] == 'left')
    _, edge, valid = advance(m, roads.store, np.array([-5., 0.]), np.eye(2), np.array([10., 0.]), JunctionConfig())
    assert valid and edge is None  # old direction gate: dot == 0 < .2*10


def test_short_connectors_are_one_maneuver_not_teleports():
    roads = Roads([
        Edge('in', [[-100, 0], [0, 0]], ('link',), 4),
        Edge('link', [[0, 0], [4, 0]], ('left', 'straight'), 4),
        Edge('left', [[4, 0], [4, 100]], (), 4),
        Edge('straight', [[4, 0], [150, 0]], (), 4),
    ])
    paths = maneuver_paths(roads.store, 'in', JunctionConfig())
    assert {m.path for m in paths} == {('link', 'left'), ('link', 'straight')}
    assert len({m.radius for m in paths}) == 1
    bank = bank_at(roads)
    feed(bank, 0., (-10, 0), (8, 0))
    assert len(bank.branches) == 2
    out = feed(bank, 1., (4, 4), (0, 8))
    assert not out['latched'] and len(bank.branches) == 2
    close(bank)


def test_missing_successor_is_unavailable_not_evidence_for_remaining_exit():
    roads = crossroad()
    roads.store.edges['in'].successors += ('not-downloaded',)
    bank = bank_at(roads)
    out = feed(bank, 0., (-20, 0), (10, 0))
    assert out['status'] == 'MAP_UNAVAILABLE' and not out['latched']
    assert not bank.branches
    close(bank)


def test_geometric_crossing_without_connection_is_not_a_candidate():
    roads = crossroad()
    roads.store.add(Edge('overpass', [[-20, -50], [-20, 50]], (), 4, layer=1))
    paths = maneuver_paths(roads.store, 'in', JunctionConfig())
    assert all('overpass' not in m.path for m in paths)


def test_bad_endpoint_connection_fails_closed():
    roads = crossroad()
    roads.store.add(Edge('bad', [[80, 50], [150, 50]], (), 4))
    roads.store.edges['in'].successors = ('bad',)
    with pytest.raises(MissingTopology):
        maneuver_paths(roads.store, 'in', JunctionConfig())


def test_maneuver_diversity_survives_many_high_weight_offset_copies():
    bank = bank_at(cap=3)
    bank.close_branches()
    bank.branches = [Branch('in', bank.base.clone(), np.array([0., i * .01]), -np.log(8))
                     for i in range(8)]
    out = feed(bank, 0., (-20, 0), (10, 0))
    assert len(bank.branches) == 3
    assert {b.maneuver.path[-1] for b in bank.branches} == {'left', 'right', 'straight'}
    assert out['pruned_mass'] > .5 and not out['latched']
    close(bank)


def test_budget_smaller_than_maneuver_count_abstains_and_releases_memory():
    bank = bank_at(cap=2)
    out = feed(bank, 0., (-20, 0), (10, 0))
    assert out['status'] == 'HYPOTHESIS_OVERFLOW'
    assert not out['latched'] and not bank.branches and bank.window.nav is None
    close(bank)


def test_branching_is_once_per_junction_not_repeated_prior_splitting():
    bank = bank_at()
    feed(bank, 0., (-20, 0), (10, 0))
    initial = {b.maneuver.path: b.logw for b in bank.branches}
    for t in range(1, 6):
        feed(bank, float(t), (-5, 0), (0, 0))
        assert len(bank.branches) == 3
        assert {b.maneuver.path: b.logw for b in bank.branches} == initial
    close(bank)


def test_way_group_cannot_certify_an_intersection_exit():
    bank = bank_at(crossroad(same_group=True))
    for t in range(5):
        out = feed(bank, float(t), (-5, 0), (10, 0))
        assert not out['latched'] and out['edge'] is None
    close(bank)


def test_recorded_window_recovers_dropped_turn_and_keeps_fallback_exact():
    bank = bank_at()
    feed(bank, 0., (-20, 0), (10, 0))
    feed(bank, 1., (0, 0), (5, 5))
    assert bank.window.nav is not None
    # Fault injection: emulate a previously premature commitment to straight.
    kept = []
    for b in bank.branches:
        if b.maneuver.path[-1] == 'straight':
            kept.append(b)
        else:
            b.nav.close()
    bank.branches = kept
    out = feed(bank, 2., (0, 20), (0, 10))
    assert out['recovery_used'] and out['recoveries'] == 1
    assert not out['latched']
    assert {b.maneuver.path[-1] for b in bank.branches} == {'left'}
    np.testing.assert_array_equal(out['base'].raw, bank.base.snapshot().raw)
    assert bank.base.count == 3  # current raw samples were not fed to fallback twice
    assert bank.window.nav is None and not bank.window.events
    for t, y in [(3., 30.), (4., 40.), (5., 50.)]:
        out = feed(bank, t, (0, y), (0, 10))
    assert out['latched'] and out['edge'] == 'left'
    close(bank)


def test_window_is_immutable_and_fails_closed_on_capacity_or_age():
    nav = FakeNav()
    for seconds, limit, second_t in [(20., 1, 2.), (1., 100, 3.)]:
        w = ReplayWindow(seconds, limit)
        w.capture(nav, 0., -np.inf, 'in', 'v1')
        row = np.array([.1, 1, 2, 3, 4, 0, 0.])
        w.append('imu', row)
        row[1] = 999.
        assert w.events[0][1][1] == 1.
        w.append('imu', np.array([second_t, 1, 2, 3, 4, 0, 0.]))
        assert w.status == 'expired' and w.nav is None and not w.events
    nav.close()


def test_expired_window_does_not_claim_recovery():
    bank = bank_at(config=replace(JunctionConfig(), recovery_s=1.0))
    feed(bank, 0., (-20, 0), (10, 0))
    out = feed(bank, 3., (80, 80), (0, 10))
    assert not out['recovery_used'] and out['recoveries'] == 0
    assert out['status'] == 'NO_CONSISTENT_ROUTE'
    close(bank)


def test_map_change_invalidates_geometry_and_replay_history():
    bank = bank_at()
    feed(bank, 0., (-20, 0), (10, 0))
    bank.roads.store.version = 'replacement'
    out = feed(bank, 1., (-10, 0), (10, 0))
    assert out['status'] == 'MAP_CHANGED' and not bank.branches
    assert bank.window.nav is None and not bank.window.events
    close(bank)


def test_overlong_connector_chain_abstains_instead_of_truncating_exit_search():
    roads = Roads([
        Edge('in', [[-100, 0], [0, 0]], ('a',), 4),
        Edge('a', [[0, 0], [4, 0]], ('b',), 4),
        Edge('b', [[4, 0], [8, 0]], ('out',), 4),
        Edge('out', [[8, 0], [100, 0]], (), 4),
    ])
    with pytest.raises(HypothesisOverflow):
        maneuver_paths(roads.store, 'in', replace(JunctionConfig(), max_hops=1))


def test_intersection_baseline_is_not_conditioned_by_any_child():
    bank = bank_at()
    control = bank.base.clone()
    for t in range(10):
        p = (-20 + t * 8., 2.)
        row = np.array([float(t), *p, 8., 0., 0., 0.])
        control.imu(row)
        bank.imu(row)
        out = bank.step(float(t))
        np.testing.assert_array_equal(out['base'].raw, control.snapshot().raw)
    close(bank)
    control.close()


@pytest.mark.parametrize('kwargs', [{'max_paths': 0}, {'radius_min_m': -1.}, {'exit_cos': 2.}, {'recovery_s': 0.}])
def test_config_validation(kwargs):
    with pytest.raises(ValueError):
        JunctionConfig(**kwargs)


def test_active_geometry_eviction_discards_checkpoint():
    bank = bank_at()
    feed(bank, 0., (-20, 0), (10, 0))
    bank.roads.store.loaded.clear()
    out = feed(bank, 1., (-10, 0), (10, 0))
    assert out['status'] == 'MAP_UNAVAILABLE' and not bank.branches
    assert bank.window.nav is None and not bank.window.events
    close(bank)


def test_replay_invalidates_out_of_order_events():
    nav = FakeNav()
    w = ReplayWindow()
    w.capture(nav, 0., 0., 'in', 'v1')
    w.append('imu', [1., 0., 0., 0., 0., 0., 0.])
    w.append('gnss', [.5] + [0.] * 13)
    assert w.status == 'out_of_order' and not w.events and w.nav is None
    nav.close()


def test_transient_branch_allocations_are_bounded():
    before = FakeNav.live
    bank = bank_at(cap=3)
    FakeNav.peak = FakeNav.live
    for t, x in enumerate((-20., -10., 0., 10., 20., 30., 40., 50.)):
        feed(bank, float(t), (x, 0.), (10., 0.))
    assert FakeNav.peak - before <= 2 * bank.cap + 4
    close(bank)
    assert FakeNav.live == before


def test_known_approach_road_is_not_a_commitment_to_a_future_exit():
    bank = bank_at()
    for t in range(3):
        out = feed(bank, float(t), (-20, 0), (0, 0))
    assert out['latched'] and out['edge'] == 'in' and out['future_exit'] is None
    assert len(bank.branches) == 3 and bank.window.nav is not None
    out = feed(bank, 3., (-5, 0), (5, 0))
    assert not out['latched'] and out['edge'] is None
    assert len(bank.branches) == 3
    close(bank)
