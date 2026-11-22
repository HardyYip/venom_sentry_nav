"""Summarize route evidence and render the measured paths on the existing map."""
from collections import defaultdict
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from PIL import Image
import yaml


root = Path(__file__).resolve().parent
results = json.loads((root / 'results.json').read_text())
contacts = json.loads((root / 'contact-summary.json').read_text())
samples = defaultdict(list)
with (root / 'samples.jsonl').open() as stream:
    for line in stream:
        row = json.loads(line)
        samples[row['leg']].append(row)
map_file = sorted(root.glob('20*/cad-mid-odin-*/map-boundary-margin.yaml'))[-1]
config = yaml.safe_load(map_file.read_text())
grid = np.asarray(Image.open(map_file.with_name(config['image'])))
resolution = config['resolution']
xmin, ymin = config['origin'][:2]
ymax = ymin + resolution * grid.shape[0]
xmax = xmin + resolution * grid.shape[1]
occupied_y, occupied_x = np.nonzero((255 - grid.astype(float)) / 255 > config['occupied_thresh'])
occupied = np.column_stack((xmin + (occupied_x + .5) * resolution,
                            ymax - (occupied_y + .5) * resolution))
footprint_half = np.array([.29, .265])
cell_radius = resolution / math.sqrt(2)


def min_clearance(positions, heading):
    lower_bound = math.inf
    for (x, y), yaw in zip(positions, heading):
        local = occupied[np.logical_and(np.abs(occupied[:, 0] - x) < 2.,
                                        np.abs(occupied[:, 1] - y) < 2.)] - [x, y]
        if not len(local):
            continue
        c, s = math.cos(yaw), math.sin(yaw)
        body_xy = local @ np.array([[c, -s], [s, c]])
        d = np.abs(body_xy) - footprint_half
        signed = np.linalg.norm(np.maximum(d, 0.), axis=1) + np.minimum(np.max(d, axis=1), 0.)
        lower_bound = min(lower_bound, float(signed.min()) - cell_radius)
    return lower_bound


fig, axes = plt.subplots(2, 4, figsize=(15.5, 7.5), constrained_layout=True, sharex=True, sharey=True)
fig.suptitle('RMUL  |  Gazebo long-route navigation', fontsize=17, fontweight='bold')
summaries = []
for n, (entry, ax) in enumerate(zip(results, axes.flat), 1):
    leg_rows = samples[entry['leg']]
    path = np.array([[row['x'], row['y']] for row in leg_rows])
    t = np.array([row['sim'] for row in leg_rows])
    peak = max(leg_rows, key=lambda row: row['speed'])
    assert math.isclose(entry['truth_peak_mps'], peak['speed'], abs_tol=1e-9)
    assert len(leg_rows) == entry['samples']
    contact_counts = {kind: sum(row['records'] for row in contacts['counts']
                                if row['leg'] == entry['leg'] and row['kind'] == kind)
                      for kind in ('ground', 'wall', 'box', 'unknown')}
    map_clearance = min_clearance(path[::25], [row['yaw'] for row in leg_rows[::25]])
    summaries.append(dict(entry, contacts=contact_counts,
                          sample_dt_median_ms=float(np.median(np.diff(t))*1000),
                          sample_dt_max_ms=float(np.max(np.diff(t))*1000),
                          approximated_map_clearance_m=map_clearance))
    ax.imshow(grid, cmap='gray', vmin=0, vmax=254, extent=(xmin, xmax, ymin, ymax),
              interpolation='nearest', alpha=.36)
    for x, y in ((3., -5.5), (6.6, -.2)):
        ax.add_patch(Rectangle((x-.29, y-.25), .58, .50, facecolor='#996515', edgecolor='#51340a'))
    ax.plot(path[:, 0], path[:, 1], color='#126782', lw=1.8)
    ax.scatter(*entry['start'], color='#15803d', marker='o', zorder=3, s=30)
    ax.scatter(*entry['goal'], color='#7c3aed', marker='x', zorder=3, s=42)
    ax.scatter(*entry['peak_at_xy'], facecolor='#e05d26', edgecolor='white', linewidth=.6, zorder=3, s=38)
    goal_label = 'arrived' if entry['goal_status'] == 4 else 'goal incomplete'
    ax.set_title(f'{n}. {entry["leg"].replace("-", " ")}\n'
                 f'{entry["direct_m"]:.1f} m direct  •  {entry["truth_peak_mps"]:.3f} m/s  •  {goal_label}',
                 fontsize=10, loc='left', pad=8)
    ax.set_xticks([0, 3, 6, 9])
    ax.set_yticks([-6, -4, -2, 0])
    ax.grid(alpha=.18, linewidth=.5)
for ax in list(axes.flat)[len(results):]:
    ax.axis('off')
    ax.text(.06, .65, '●  Start', color='#15803d', transform=ax.transAxes)
    ax.text(.06, .52, '×  Goal', color='#7c3aed', transform=ax.transAxes)
    ax.text(.06, .39, '●  Peak-speed sample', color='#e05d26', transform=ax.transAxes)
    ax.text(.06, .26, '■  Fixed box', color='#996515', transform=ax.transAxes)
fig.text(.50, .012, 'Map X (m)', ha='center')
fig.text(.006, .52, 'Map Y (m)', va='center', rotation=90)
fig.savefig(root / 'trajectories.png', dpi=145)
report = {
    'result_count': len(summaries),
    'contacts_decode_errors': contacts['malformed'],
    'map_clearance_method': 'conservative proxy from 5 cm navigation occupancy cells, sample stride 25, body rectangle 0.58 x 0.53 m; excludes the two spawned boxes',
    'legs': summaries,
}
(root / 'review.json').write_text(json.dumps(report, indent=2) + '\n')
