"""Partial-cache review regressions: absence of map data is not route evidence."""
import numpy as np
import pytest
from test_relocalization import IncrementNav, straight
from test_intersections import Roads
from routefusion.tiles import Edge
from route_recovery import RelocalizingRoadBank, RelocalizationConfig, inconsistent_edges
from intersections import MissingTopology


def border_roads(parallel=True):
    # Receiver at y=250 has its own tile loaded, while the competing northern
    # road at y=270 lies only in the neighboring tile (tile_size=256 m).
    edges=[Edge('south',[[-200.,230.],[500.,230.]],(),6.)]
    if parallel:edges.append(Edge('north',[[-200.,270.],[500.,270.]],(),6.))
    roads=Roads(edges)
    roads.store.cover([-250.,200.],[550.,310.])
    return roads


def test_missing_neighbor_cannot_create_a_false_confident_route():
    roads=border_roads()
    roads.store.loaded={key for key in roads.store.loaded if key[1]==0}
    bank=RelocalizingRoadBank(IncrementNav((0.,250.)),roads)
    control=bank.base.clone()
    try:
        assert roads.store.is_loaded(np.array([0.,250.]))
        for t in range(8):
            if t:
                row=np.array([float(t),0,0,0,0,0,0])
                bank.imu(row);control.imu(row)
            out=bank.step(float(t))
            assert out['status']=='MAP_UNAVAILABLE' and not out['latched']
            assert not bank.loss.used and bank.loss.count==0
            np.testing.assert_array_equal(bank.base.snapshot().raw,control.snapshot().raw)
    finally:bank.reset();bank.base.close();control.close()


def test_neighbor_restoration_preserves_parallel_ambiguity():
    roads=border_roads();loaded=roads.store.loaded.copy()
    roads.store.loaded={key for key in loaded if key[1]==0}
    bank=RelocalizingRoadBank(IncrementNav((0.,250.)),roads)
    try:
        for t in range(10):
            if t:bank.imu(np.array([float(t),0,0,0,0,0,0]))
            if t==3:roads.store.loaded=loaded
            out=bank.step(float(t))
            assert not out['latched']
        assert bank.loss.activation_t==5.
        assert {b.edge for b in bank.branches}=={'south','north'}
        assert .49<out['top_weight']<.51
    finally:bank.reset();bank.base.close()


def test_partial_cache_clears_previous_confirmation_evidence():
    roads=border_roads(False);loaded=roads.store.loaded.copy()
    bank=RelocalizingRoadBank(IncrementNav((0.,250.)),roads)
    try:
        for t in range(6):
            if t:bank.imu(np.array([float(t),0,0,0,0,0,0]))
            if t==2:roads.store.loaded={key for key in loaded if key[1]==0}
            if t==3:roads.store.loaded=loaded
            out=bank.step(float(t))
            if t==2:assert out['status']=='MAP_UNAVAILABLE' and bank.loss.count==0
            if t<5:assert not bank.loss.used
        assert bank.loss.activation_t==5.
    finally:bank.reset();bank.base.close()


def test_loaded_boundary_tile_does_not_imply_extract_coverage():
    roads=straight();roads.store.cover([-600.,-600.],[700.,700.])
    # The receiver is inside the extract, but the 36 m search radius extends
    # north of the captured bounding latitude. All relevant tiles are loaded.
    roads.bbox=np.array([-.002,-.005,.0002,.005])
    n=IncrementNav()
    try:
        with pytest.raises(MissingTopology,match='declared map extract'):
            inconsistent_edges(n.snapshot(),roads,RelocalizationConfig())
    finally:n.close()
