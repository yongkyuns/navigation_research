"""Keep the human-readable synthetic results aligned with retained evidence."""
import json
from pathlib import Path


def test_documented_synthetic_results_match_retained_evidence():
    root = Path(__file__).resolve().parents[1]
    evidence = json.loads((root / 'map_correlation_evidence.json').read_text())['results']
    document = (root / 'MAP_CORRELATION.md').read_text()
    models = {
        'GNSS only': 'GNSS only',
        'Naively independent map': 'naive independent centerline',
        'Per-update inflated map noise': 'independent inflated noise',
        'Schmidt shared map offset': 'correlated Schmidt map offset',
    }
    rows = {}
    for line in document.splitlines():
        if not line.startswith('|'):
            continue
        cells = [cell.strip().replace('**', '') for cell in line.strip('|').split('|')]
        if cells[0] in models:
            assert cells[0] not in rows, 'Duplicate result row'
            rows[cells[0]] = cells[1:]
    assert set(rows) == set(models), 'Missing synthetic result row'
    for label, name in models.items():
        value = evidence[name]
        expected = [f"{value['position_rmse_m']:.3f}",
                    f"{value['mean_predicted_sigma_m']:.3f}",
                    f"{value['mean_position_nees']:.0f}" if value['mean_position_nees'] >= 10
                    else f"{value['mean_position_nees']:.2f}",
                    '–' if value['map_updates_per_trial'] == 0
                    else f"{value['p95_road_update_step_m']:.3f}"]
        assert rows[label] == expected, f'{label}: documentation differs from retained evidence'
