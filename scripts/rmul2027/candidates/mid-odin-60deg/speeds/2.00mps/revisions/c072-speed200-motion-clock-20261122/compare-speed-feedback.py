"""Compare original-clock and source-clock odometry with synchronized Gazebo truth."""
from bisect import bisect_left, bisect_right
import json
import math
from pathlib import Path
import statistics
import sys


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def metrics(errors):
    return {
        'paired_samples': len(errors),
        'mean_error_mps': statistics.fmean(errors),
        'mae_mps': statistics.fmean(abs(error) for error in errors),
        'rmse_mps': math.sqrt(statistics.fmean(error * error for error in errors)),
        'peak_absolute_error_mps': max(abs(error) for error in errors),
    }


def analyze(directory):
    truth = read_rows(directory / 'samples.jsonl')
    feedback = read_rows(directory / 'odometry-feedback.jsonl')
    stamps = [sample['sim'] for sample in truth]
    results = json.loads((directory / 'results.json').read_text())
    errors = {result['leg']: [] for result in results}
    for sample in feedback:
        name, stamp = sample['leg'], sample['sim']
        if name not in errors:
            continue
        start = bisect_left(stamps, stamp - .025)
        end = bisect_right(stamps, stamp + .025)
        neighbors = [sample['speed'] for sample in truth[start:end] if sample['leg'] == name]
        if len(neighbors) < 10:
            continue
        speed = statistics.median(neighbors)
        if speed >= .15:
            errors[name].append(math.hypot(*sample['v']) - speed)
    return {
        'evidence_dir': str(directory),
        'per_leg': {name: metrics(samples) for name, samples in errors.items()},
        'combined': metrics([error for samples in errors.values() for error in samples]),
        'route_results': results,
    }


if __name__ == '__main__':
    original, corrected, output = (Path(arg).resolve() for arg in sys.argv[1:])
    result = {
        'method': 'Compare same-leg odometry speed against the median Gazebo ground-truth speed within ±25 ms of each odometry header stamp, with ground-truth speed at least 0.15 m/s.',
        'callback_clock': analyze(original),
        'source_clock': analyze(corrected),
    }
    previous = result['callback_clock']['combined']['mae_mps']
    current = result['source_clock']['combined']['mae_mps']
    result['relative_mae_reduction_percent'] = 100. * (previous - current) / previous
    output.write_text(json.dumps(result, indent=2) + '\n')
    print(f'MAE {previous:.4f} -> {current:.4f} m/s; reduction {result["relative_mae_reduction_percent"]:.1f}%')
