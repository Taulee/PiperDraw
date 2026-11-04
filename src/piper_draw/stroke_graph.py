from collections import deque
from itertools import combinations

import numpy as np


def _neighbors(mask):
    pixels = set(map(tuple, np.argwhere(mask)))
    return {
        (y, x): sorted(
            (y + dy, x + dx)
            for dy in (-1, 0, 1)
            for dx in (-1, 0, 1)
            if (dy or dx)
            and (y + dy, x + dx) in pixels
            # 拐角已有正交连接时，对角边会产生虚假的小三角和分叉。
            and not (dy and dx and ((y + dy, x) in pixels or (y, x + dx) in pixels))
        )
        for y, x in sorted(pixels)
    }


def _trace_graph(neighbors):
    used = set()

    def walk(start, second):
        points = [start, second]
        used.add(tuple(sorted((start, second))))
        previous, current = start, second
        while current != start and len(neighbors[current]) == 2:
            following = next(p for p in neighbors[current] if p != previous)
            edge = tuple(sorted((current, following)))
            if edge in used:
                break
            used.add(edge)
            points.append(following)
            previous, current = current, following
        return points

    chains = []
    terminals = [p for p in neighbors if len(neighbors[p]) != 2]
    for start in terminals + list(neighbors):
        for second in neighbors[start]:
            if tuple(sorted((start, second))) not in used:
                points = walk(start, second)
                protected = len(neighbors[points[0]]) > 2 and len(neighbors[points[-1]]) > 2
                chains.append((points, protected))
    return chains


def trace(mask):
    """按无向边追踪骨架，在端点和分叉处分笔；坐标顺序为 [y, x]。"""
    return [
        (np.asarray(points), points[0] == points[-1], protected)
        for points, protected in _trace_graph(_neighbors(mask))
    ]


def _junction_routes(neighbors, pixel_size_mm, junction_span_mm):
    junctions = {p for p in neighbors if len(neighbors[p]) > 2}
    remaining = set(junctions)
    groups = []
    # 只收缩很小的相邻分叉簇；大范围密集结构仍保留原图连接关系。
    max_span = max(2.0, min(4.0, junction_span_mm / pixel_size_mm))
    for seed in sorted(junctions):
        if seed not in remaining:
            continue
        remaining.remove(seed)
        group = {seed}
        queue = deque([seed])
        while queue:
            current = queue.popleft()
            for following in neighbors[current]:
                if following in remaining:
                    remaining.remove(following)
                    group.add(following)
                    queue.append(following)
        if np.ptp(np.asarray(list(group)), axis=0).max() <= max_span:
            groups.append(group)
        else:
            groups.extend({p} for p in sorted(group))

    routes = {}
    for group in groups:
        center = np.mean(list(group), axis=0)
        representative = min(group, key=lambda p: (np.linalg.norm(p - center), p))
        routes[representative] = [representative]
        queue = deque([representative])
        while queue:
            current = queue.popleft()
            for following in neighbors[current]:
                if following in group and following not in routes:
                    routes[following] = routes[current] + [following]
                    queue.append(following)
    return routes


def _outward_direction(points, pixel_size_mm, direction_lookahead_mm):
    points = np.asarray(points, dtype=float)
    # 避开交汇处单个像素的方向噪声，同时只看支路开头的短段。
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    lookahead = max(3.0, direction_lookahead_mm / pixel_size_mm)
    index = min(int(np.searchsorted(np.cumsum(lengths), lookahead)) + 1, len(points) - 1)
    direction = points[index] - points[0]
    norm = np.linalg.norm(direction)
    if norm == 0:
        return None
    return direction / norm


def _pair_ports(ports, directions, join_angle_deg):
    candidates = {port: [] for port in ports}
    for first, second in combinations(ports, 2):
        if directions[first] is None or directions[second] is None:
            continue
        angle = np.degrees(np.arccos(np.clip(-directions[first] @ directions[second], -1, 1)))
        if angle <= join_angle_deg:
            candidates[first].append((angle, second))
            candidates[second].append((angle, first))

    choices = {}
    ambiguity = min(10.0, join_angle_deg / 3)
    for port, options in candidates.items():
        options.sort()
        if options and (len(options) == 1 or options[1][0] - options[0][0] > ambiguity):
            choices[port] = options[0][1]
    # 两侧都必须认为对方是明显最优，避免在复杂交汇处任意选一条支路。
    return {port: other for port, other in choices.items() if choices.get(other) == port}


def trace_strokes(
    mask,
    pixel_size_mm,
    min_stroke_mm,
    join_angle_deg=35.0,
    direction_lookahead_mm=0.8,
    junction_span_mm=0.5,
):
    """归并微小交汇簇，按局部方向接续；不跨空白补线或删除真实短桥。"""
    neighbors = _neighbors(mask)
    routes = _junction_routes(neighbors, pixel_size_mm, junction_span_mm)
    chains = []
    ports_by_junction = {}
    directions = {}
    for points, protected in _trace_graph(neighbors):
        first, last = points[0], points[-1]
        if first in routes and last in routes and routes[first][0] == routes[last][0]:
            representative = routes[first][0]
            if all(p in routes and routes[p][0] == representative for p in points):
                continue
        # 连接代表点时沿簇内现有边行走，避免在白色背景上凭空增加连线。
        prefix = routes.get(first, [first])[:-1]
        suffix = routes.get(last, [last])[::-1][1:]
        chain_id = len(chains)
        chains.append((prefix + points + suffix, protected))
        for side, endpoint, oriented in ((0, first, points), (1, last, points[::-1])):
            if endpoint in routes:
                port = (chain_id, side)
                junction = routes[endpoint][0]
                ports_by_junction.setdefault(junction, []).append(port)
                directions[port] = _outward_direction(
                    oriented, pixel_size_mm, direction_lookahead_mm
                )

    pairs = {}
    for ports in ports_by_junction.values():
        pairs.update(_pair_ports(ports, directions, join_angle_deg))

    used = set()
    strokes = []
    # 先走开放笔画，剩下的成对端口构成闭环。
    ports = [(i, side) for i in range(len(chains)) for side in (0, 1)]
    starts = [port for port in ports if port not in pairs] + ports
    for start in starts:
        if start[0] in used:
            continue
        points = []
        protected = False
        current = start
        while current[0] not in used:
            chain_id, side = current
            chain_points, chain_protected = chains[chain_id]
            oriented = chain_points if side == 0 else chain_points[::-1]
            points.extend(oriented if not points else oriented[1:])
            protected |= chain_protected
            used.add(chain_id)
            following = pairs.get((chain_id, 1 - side))
            if following is None:
                break
            current = following
        array = np.asarray(points)
        length = np.linalg.norm(np.diff(array, axis=0), axis=1).sum() * pixel_size_mm
        if length >= min_stroke_mm or protected:
            strokes.append((array, points[0] == points[-1], protected))
    return strokes
