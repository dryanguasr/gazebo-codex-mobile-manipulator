"""Planar mobile transport with bounded commands and explicit simulated localization."""

import math


TRANSPORT_STATES = ('NAVIGATE', 'DOCK_BASE')


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def model_planar_pose(message):
    """Model-only PosePublisher deliberately publishes exactly one transform."""
    if len(message.transforms) != 1:
        return None
    t = message.transforms[0].transform
    pose = (float(t.translation.x), float(t.translation.y), yaw_from_quaternion(t.rotation))
    return pose if all(math.isfinite(v) for v in pose) else None


def world_to_odom(world_base, odom_base):
    """Compute world_T_odom from two estimates of the same base footprint."""
    yaw = wrap_angle(world_base[2] - odom_base[2])
    c, s = math.cos(yaw), math.sin(yaw)
    return (
        world_base[0] - c * odom_base[0] + s * odom_base[1],
        world_base[1] - s * odom_base[0] - c * odom_base[1],
        yaw,
    )


def slew(previous, target, acceleration, dt):
    step = acceleration * max(0.0, min(dt, 0.1))
    return previous + max(-step, min(step, target - previous))


class PlanarRoute:
    """Stop at each waypoint, then align yaw; never command lateral motion."""

    def __init__(self, waypoints, speed=0.06, yaw_speed=0.20,
                 acceleration=0.025, yaw_acceleration=0.12):
        if not waypoints or any(len(p) != 3 for p in waypoints):
            raise ValueError('Expected nonempty [x, y, yaw] waypoints')
        if not all(math.isfinite(v) for p in waypoints for v in p):
            raise ValueError('Waypoints must be finite')
        if min(speed, yaw_speed, acceleration, yaw_acceleration) <= 0:
            raise ValueError('Motion limits must be positive')
        self.waypoints = waypoints
        self.speed, self.yaw_speed = speed, yaw_speed
        self.acceleration, self.yaw_acceleration = acceleration, yaw_acceleration
        self.index = 0
        self.aligning = False
        self.v = self.w = 0.0
        self.turn_start = None
        self.pivot_offset = (0.0, 0.0)

    def step(self, pose, dt):
        if self.index >= len(self.waypoints):
            return 0.0, 0.0, True
        x, y, yaw = pose
        tx, ty, tyaw = self.waypoints[self.index]
        # Identify the effective skid-steer pivot from completed pure turns.
        # At the final station, target that pivot so yaw alignment does not
        # displace the base center away from its requested docking position.
        if self.index == len(self.waypoints) - 1 and not self.aligning:
            rx, ry = self.pivot_offset
            tx += ((math.cos(yaw) - math.cos(tyaw)) * rx
                   - (math.sin(yaw) - math.sin(tyaw)) * ry)
            ty += ((math.sin(yaw) - math.sin(tyaw)) * rx
                   + (math.cos(yaw) - math.cos(tyaw)) * ry)
        distance = math.hypot(tx - x, ty - y)
        if distance <= 0.002 and not self.aligning:
            self.aligning = True
            self.turn_start = pose
        if self.aligning:
            error = wrap_angle(tyaw - yaw)
            target_v = 0.0
            target_w = max(-self.yaw_speed, min(self.yaw_speed, 1.2 * error))
            if abs(error) <= 0.004 and abs(self.v) < 0.002 and abs(self.w) < 0.006:
                if self.turn_start is not None:
                    sx, sy, syaw = self.turn_start
                    a = math.cos(yaw) - math.cos(syaw)
                    b = math.sin(yaw) - math.sin(syaw)
                    denominator = a * a + b * b
                    if denominator > 0.001:
                        rx = (a * (x - sx) + b * (y - sy)) / denominator
                        ry = (-b * (x - sx) + a * (y - sy)) / denominator
                        if math.hypot(rx, ry) <= 0.2:
                            self.pivot_offset = (rx, ry)
                self.index += 1
                self.aligning = False
                self.v = self.w = 0.0
                return 0.0, 0.0, self.index == len(self.waypoints)
        else:
            error = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
            target_w = max(-self.yaw_speed, min(self.yaw_speed, 1.2 * error))
            target_v = min(self.speed, 0.8 * distance) if abs(error) < 0.12 else 0.0
        self.v = slew(self.v, target_v, self.acceleration, dt)
        self.w = slew(self.w, target_w, self.yaw_acceleration, dt)
        return self.v, self.w, False


class SmoothRoute:
    """Continuous rounded L route, followed by a straight precision approach.

    This exercise starts at (0, 0, 0) and finishes facing +Y. The quintic
    Bezier corner has zero endpoint curvature, matching both straight legs.
    Pure pursuit uses actual model localization; wheels still drive the base.
    """

    def __init__(self, goal, speed=0.10, yaw_speed=0.45,
                 acceleration=0.05, yaw_acceleration=0.30, radius=0.35):
        if (len(goal) != 3 or not all(math.isfinite(v) for v in goal)
                or abs(wrap_angle(goal[2] - math.pi / 2)) > 1e-6):
            raise ValueError('Smooth route requires a finite +Y docking goal')
        if (not all(math.isfinite(v) and v > 0 for v in
                    (speed, yaw_speed, acceleration, yaw_acceleration, radius))
                or min(goal[:2]) <= radius):
            raise ValueError('Positive limits and room for the rounded corner required')
        self.goal = tuple(goal)
        self.speed, self.yaw_speed = speed, yaw_speed
        self.acceleration, self.yaw_acceleration = acceleration, yaw_acceleration
        self.v = self.w = 0.0
        self.index = 0
        self.pivot_offset = (0.0, 0.0)
        self.finished = False
        self.aligning = False
        gx, gy, _ = goal
        start = gx - radius
        controls = [(start, 0), (start + .4 * radius, 0),
                    (start + .8 * radius, 0), (gx, .2 * radius),
                    (gx, .6 * radius), (gx, radius)]
        self.path = [(start * i / 60, 0.0) for i in range(61)]
        for i in range(1, 101):
            t = i / 100
            self.path.append(tuple(sum(
                math.comb(5, k) * (1 - t) ** (5 - k) * t ** k * p[j]
                for k, p in enumerate(controls)) for j in (0, 1)))
        self.path += [(gx, radius + (gy - radius) * i / 60) for i in range(1, 61)]
        self.control_offset = 0.06
        points = self.path
        self.path = []
        for i, point in enumerate(points):
            a, b = points[max(0, i - 1)], points[min(len(points) - 1, i + 1)]
            heading = math.atan2(b[1] - a[1], b[0] - a[0])
            self.path.append((point[0] - self.control_offset * math.cos(heading),
                              point[1] - self.control_offset * math.sin(heading)))
        self.nearest = 0

    def step(self, pose, dt):
        if self.finished:
            return 0.0, 0.0, True
        base_x, base_y, yaw = pose
        x = base_x - self.control_offset * math.cos(yaw)
        y = base_y - self.control_offset * math.sin(yaw)
        gx, gy, gyaw = self.goal
        self.nearest = min(
            range(self.nearest, min(len(self.path), self.nearest + 40)),
            key=lambda i: math.dist((x, y), self.path[i]),
        )
        self.index = min(2, self.nearest // 100)
        lookahead = 0.12
        target_index = self.nearest
        arc = 0.0
        while target_index + 1 < len(self.path) and arc < lookahead:
            arc += math.dist(self.path[target_index], self.path[target_index + 1])
            target_index += 1
        tx, ty = self.path[target_index]
        remaining = gy - base_y
        # Extend the final tangent, avoiding unstable atan2 at a tiny goal error.
        final_leg = self.nearest >= 161
        if final_leg:
            tx, ty = gx, y + lookahead
        heading = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
        target_v = self.speed / (1.0 + 1.5 * abs(heading))
        if final_leg:
            target_v = min(target_v, max(0.0, 0.9 * remaining))
        target_w = 2.0 * max(target_v, 0.025) * math.sin(heading) / max(
            0.03, math.dist((x, y), (tx, ty)))
        if final_leg and remaining <= 0.0015:
            self.aligning = True
        if self.aligning:
            target_v = 0.0
            target_w = 1.2 * wrap_angle(gyaw - yaw)
        target_w = max(-self.yaw_speed, min(self.yaw_speed, target_w))
        self.v = slew(self.v, target_v, self.acceleration, dt)
        self.w = slew(self.w, target_w, self.yaw_acceleration, dt)
        if (self.aligning and math.dist((base_x, base_y), (gx, gy)) <= 0.004
                and abs(wrap_angle(gyaw - yaw)) <= 0.004
                and abs(self.v) < 0.0001 and abs(self.w) < 0.005):
            self.finished = True
            self.index = 3
            return 0.0, 0.0, True
        return self.v, self.w, False


def relative_quaternion(parent, child):
    """Quaternion parent^-1 * child, tuples ordered x, y, z, w."""
    ax, ay, az, aw = (-parent[0], -parent[1], -parent[2], parent[3])
    bx, by, bz, bw = child
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_distance_deg(first, second):
    norm = math.sqrt(sum(v * v for v in first) * sum(v * v for v in second))
    if norm < 1e-12 or not math.isfinite(norm):
        return math.inf
    dot = abs(sum(a * b for a, b in zip(first, second))) / norm
    return math.degrees(2.0 * math.acos(min(1.0, dot)))
