"""Create the fixed-heading planning grid and validate straight path sections."""
import math
from dataclasses import dataclass

import numpy as np
from PIL import Image
from scipy.ndimage import maximum_filter
import yaml


@dataclass
class PlanningGrid:
    blocked: np.ndarray
    origin: tuple[float, float]
    resolution: float

    def segment_is_free(self, start, end):
        steps = max(1, math.ceil(math.dist(start, end) / (self.resolution / 4)))
        points = np.linspace(start, end, steps + 1)
        indices = np.floor((points - self.origin) / self.resolution).astype(int)
        xs, ys = indices.T
        height, width = self.blocked.shape
        if np.any((xs < 0) | (xs >= width) | (ys < 0) | (ys >= height)):
            return False
        return not self.blocked[ys, xs].any()


def make_planning_map(source, output, controller_params, clearance_margin=.025):
    info = yaml.safe_load(source.read_text())
    if info['mode'] != 'trinary' or info['negate'] != 0 or info['origin'][2] != 0:
        raise ValueError('The planning map requires an unrotated trinary occupancy image')
    with Image.open(source.parent / info['image']) as image:
        occupancy = 1.0 - np.asarray(image.convert('L'), dtype=float) / 255
    blocked = occupancy >= info['free_thresh']
    config = yaml.safe_load(controller_params.read_text())
    footprint = yaml.safe_load(config['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    half_x, half_y = np.max(np.abs(footprint), axis=0)
    resolution = info['resolution']
    pad_x = math.ceil((half_x + clearance_margin) / resolution - 1e-9)
    pad_y = math.ceil((half_y + clearance_margin) / resolution - 1e-9)
    expanded = maximum_filter(blocked, size=(2 * pad_y + 1, 2 * pad_x + 1),
                              mode='constant', cval=True)
    image_path = output.with_suffix('.pgm')
    Image.fromarray(np.where(expanded, 0, 254).astype(np.uint8)).save(image_path)
    info['image'] = image_path.name
    output.write_text(yaml.safe_dump(info, sort_keys=False))
    return PlanningGrid(np.flipud(expanded), tuple(info['origin'][:2]), resolution)


def split_at_turns(path, grid, tolerance=.012):
    """Simplify sampled path vertices while staying on the footprint-safe grid."""
    points = [path[0]]
    for point in path[1:]:
        if math.dist(point, points[-1]) > 1e-5:
            points.append(point)
    if len(points) < 2:
        raise ValueError('A planned route must have distinct start and goal positions')
    points = np.asarray(points)
    waypoints = [points[0].tolist()]
    anchor = 0
    while anchor < len(points) - 1:
        start = points[anchor]
        next_index = anchor + 1
        if not grid.segment_is_free(start, points[next_index]):
            raise RuntimeError(f'ThetaStar path crosses the footprint limit at vertex {anchor}')
        for candidate in range(next_index + 1, len(points)):
            delta = points[candidate] - start
            length_squared = delta @ delta
            if length_squared < 1e-12:
                break
            offsets = points[anchor + 1:candidate] - start
            fractions = np.clip((offsets @ delta) / length_squared, 0., 1.)
            deviation = np.linalg.norm(offsets - fractions[:, None] * delta, axis=1).max()
            if deviation > tolerance or not grid.segment_is_free(start, points[candidate]):
                break
            next_index = candidate
        waypoints.append(points[next_index].tolist())
        anchor = next_index
    return waypoints
