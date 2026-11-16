"""ABI comparison regressions: allocation storage is not navigation state."""
from types import SimpleNamespace
import numpy as np
import pytest
from verify_native_transition import compare_navigation_snapshots


def snapshot(size=4096.):
    raw = np.zeros(358, dtype=np.float64)
    raw[7] = size
    return SimpleNamespace(raw=raw)


def test_only_allocation_metadata_may_differ():
    a, b = snapshot(), snapshot(5400.)
    # The old full-array equality fails even though all navigation fields match.
    assert not np.array_equal(a.raw, b.raw)
    assert compare_navigation_snapshots(a, b) == (4096, 5400)


def test_every_other_snapshot_field_is_still_compared_exactly():
    a = snapshot()
    for index in range(358):
        if index == 7:
            continue
        b = snapshot(5400.)
        b.raw[index] = np.nextafter(b.raw[index], np.inf)
        with pytest.raises(AssertionError, match='navigation snapshot differs'):
            compare_navigation_snapshots(a, b)


def test_signed_zero_differences_are_not_tolerance_hidden():
    a, b = snapshot(), snapshot(5400.)
    b.raw[25] = -0.0
    assert np.array_equal(a.raw[:7], b.raw[:7])
    with pytest.raises(AssertionError, match='navigation snapshot differs'):
        compare_navigation_snapshots(a, b)


@pytest.mark.parametrize('size', [0., -1., .5, np.nan, np.inf])
def test_invalid_allocation_metadata_is_not_ignored(size):
    with pytest.raises(AssertionError, match='allocation-size metadata'):
        compare_navigation_snapshots(snapshot(), snapshot(size))


@pytest.mark.parametrize('raw', [np.zeros(357), np.zeros(359), np.zeros((358, 1)),
                                 np.zeros(358, dtype=np.float32)])
def test_unknown_snapshot_abi_is_rejected(raw):
    with pytest.raises(AssertionError, match='snapshot ABI'):
        compare_navigation_snapshots(snapshot(), SimpleNamespace(raw=raw))


def test_preinitialization_timestamp_sentinel_can_match_bitwise():
    a, b = snapshot(), snapshot(5400.)
    a.raw[5] = b.raw[5] = -np.inf
    assert compare_navigation_snapshots(a, b) == (4096, 5400)
