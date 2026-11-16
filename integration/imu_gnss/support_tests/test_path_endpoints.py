"""Review regressions: explicit topology is not proof of identical geometry."""
import numpy as np
import pytest

from path_support import PathIndex, CausalSupport
from routefusion.tiles import Edge
from supported_map import SupportedMapEKF
from support_tests.test_support import roads, Snapshot, Nav, same


@pytest.mark.parametrize('axis', [0, 1])
@pytest.mark.parametrize('sign', [-1., 1.])
@pytest.mark.parametrize('gap_m', [1e-9, 1e-8, 1e-7, 9e-7, 2e-6])
def test_nonidentical_endpoints_are_not_snapped_or_bridged(axis, sign, gap_m):
    endpoint = np.array([50., 0.])
    other = endpoint.copy()
    other[axis] += sign * gap_m
    a = Edge('a', [[0., 0.], endpoint], ('b',))
    b = Edge('b', [other, [100., 10.]])
    original = [edge.points.copy() for edge in (a, b)]
    index = PathIndex(roads([a, b]))
    # Endpoint mismatches must fail closed regardless of which side of Edge's
    # minimum segment length they lie on. The input graph remains unchanged.
    assert index.by_edge['a'] is not index.by_edge['b']
    assert index.by_edge['a'].edges == ('a',)
    assert index.by_edge['b'].edges == ('b',)
    for edge, before in zip((a, b), original):
        np.testing.assert_array_equal(edge.points, before)


def test_a_refused_geometry_join_only_suspends_the_measurement():
    r = roads([Edge('a', [[0., 0.], [50., 0.]], ('b',)),
               Edge('b', [[50., 1e-7], [100., 10.]])])
    selector = CausalSupport(PathIndex(r))
    filt = SupportedMapEKF(Nav((35., 8.)), r)
    native = filt.nav
    path, support = selector.select(1., Snapshot((35., 1.)), 'a')
    _, update = filt.step(1., path, support)
    assert update['updated']
    # The raw state has crossed a geometrically unverified connection. Without
    # a newly accepted seed the old path cannot supply an outgoing constraint.
    native.s.p[0] = 55.
    before = native.snapshot()
    path, support = selector.select(2., Snapshot((55., 1.)), None)
    after, skipped = filt.step(2., path, support)
    assert path is None and support['reason'] == 'OUTSIDE_PATH'
    assert not skipped['updated'] and not skipped['reset']
    assert filt.nav is native
    same(before, after)
    filt.feed('imu', [3.])
    assert native.snapshot().p[0] == before.p[0] + 1.
