from __future__ import annotations

from typing import List

from src.models import CarPose, Cone, Path2D


class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and a list of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.
    """
    
    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT.
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.
        """
        import math

        start = (self.car_pose.x, self.car_pose.y)

        blue = [(c.x, c.y) for c in self.cones if c.color == 1]
        yellow = [(c.x, c.y) for c in self.cones if c.color == 0]

        # Assumptions (mention these in the README)
        TRACK_WIDTH = 2.0       # assumed distance between blue and yellow boundaries (m)
        PATH_LENGTH = 5.0       # minimum path length (m)
        MAX_PATH_LENGTH = 10.0  # maximum path length (m)
        STEP = 0.5              # maximum distance between two path points (m)

        # ---------- small helper functions ----------

        def distance(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        def unit_vector(v):
            """Same direction as v, but with length 1."""
            length = math.hypot(v[0], v[1])
            if length < 1e-9:
                return (1.0, 0.0)
            return (v[0] / length, v[1] / length)

        def fit_line(points):
            """Fit a straight line through the points.
            Returns a point on the line (the average) and the line's direction."""
            mean_x = sum(p[0] for p in points) / len(points)
            mean_y = sum(p[1] for p in points) / len(points)

            xx = sum((p[0] - mean_x) ** 2 for p in points)
            yy = sum((p[1] - mean_y) ** 2 for p in points)
            xy = sum((p[0] - mean_x) * (p[1] - mean_y) for p in points)

            if xx + yy < 1e-9:
                return (mean_x, mean_y), (1.0, 0.0)

            angle = 0.5 * math.atan2(2.0 * xy, xx - yy)
            direction = (math.cos(angle), math.sin(angle))
            return (mean_x, mean_y), direction

        # ---------- step 1: choose the key points (waypoints) ----------

        if blue and yellow:
            # CASE: cones on both sides -> go through the middle of each blue/yellow pair.
            remaining_blue = blue.copy()
            remaining_yellow = yellow.copy()
            center_points = []

            while remaining_blue and remaining_yellow:
                # Pair the blue and yellow cone that are closest to each other.
                best_pair = min(
                    (
                        (i, j)
                        for i in range(len(remaining_blue))
                        for j in range(len(remaining_yellow))
                    ),
                    key=lambda pair: distance(
                        remaining_blue[pair[0]],
                        remaining_yellow[pair[1]]
                    )
                )
                blue_index, yellow_index = best_pair

                b = remaining_blue.pop(blue_index)
                y = remaining_yellow.pop(yellow_index)

                midpoint = ((b[0] + y[0]) / 2.0, (b[1] + y[1]) / 2.0)
                center_points.append(midpoint)

            # Visit the midpoints from nearest to farthest.
            center_points.sort(key=lambda p: distance(start, p))
            waypoints = [start] + center_points

        elif blue or yellow:
            # CASE: only one side visible.
            visible_side = blue if blue else yellow
            is_blue = bool(blue)

            if len(visible_side) == 1:
                # One cone: use the car's yaw as the travel direction.
                cone = visible_side[0]
                direction = (math.cos(self.car_pose.yaw), math.sin(self.car_pose.yaw))

                # Blue is on the left, so move to the right of it.
                # Yellow is on the right, so move to the left of it.
                if is_blue:
                    inward = (direction[1], -direction[0])
                else:
                    inward = (-direction[1], direction[0])
                inward = unit_vector(inward)

                # Move half a track width toward the middle of the track.
                target = (
                    cone[0] + inward[0] * (TRACK_WIDTH / 2.0),
                    cone[1] + inward[1] * (TRACK_WIDTH / 2.0)
                )

                # Continue forward from that shifted point.
                target = (
                    target[0] + direction[0] * PATH_LENGTH,
                    target[1] + direction[1] * PATH_LENGTH
                )
                waypoints = [start, target]

            else:
                # Two or more cones on one side: fit a line through them.
                line_center, direction = fit_line(visible_side)

                # Make the line point the same general way the car is facing.
                car_direction = (math.cos(self.car_pose.yaw), math.sin(self.car_pose.yaw))
                if (direction[0] * car_direction[0] + direction[1] * car_direction[1]) < 0:
                    direction = (-direction[0], -direction[1])

                # Blue -> shift right, yellow -> shift left (toward the track middle).
                if is_blue:
                    normal = (direction[1], -direction[0])
                else:
                    normal = (-direction[1], direction[0])
                normal = unit_vector(normal)

                centerline_point = (
                    line_center[0] + normal[0] * (TRACK_WIDTH / 2.0),
                    line_center[1] + normal[1] * (TRACK_WIDTH / 2.0)
                )

                target = (
                    centerline_point[0] + direction[0] * 4.0,
                    centerline_point[1] + direction[1] * 4.0
                )
                waypoints = [start, target]

        else:
            # CASE: no cones -> go straight ahead along the car's yaw.
            direction = (math.cos(self.car_pose.yaw), math.sin(self.car_pose.yaw))
            waypoints = [
                start,
                (
                    start[0] + direction[0] * PATH_LENGTH,
                    start[1] + direction[1] * PATH_LENGTH
                )
            ]

        # ---------- step 2: fill in points every <= 0.5 m ----------

        path = []
        current = waypoints[0]

        for target in waypoints[1:]:
            segment_length = distance(current, target)
            steps = max(1, math.ceil(segment_length / STEP))

            for i in range(1, steps + 1):
                t = i / steps
                point = (
                    current[0] + (target[0] - current[0]) * t,
                    current[1] + (target[1] - current[1]) * t
                )

                # Do not allow the path to exceed 10 m.
                if distance(start, point) > MAX_PATH_LENGTH:
                    break

                path.append(point)

            current = path[-1] if path else current

            if distance(start, current) >= MAX_PATH_LENGTH:
                break

        # ---------- step 3: make sure the path is at least 5 m ----------

        if path:
            current_length = distance(start, path[-1])

            if current_length < PATH_LENGTH:
                if len(path) >= 2:
                    direction = unit_vector(
                        (path[-1][0] - path[-2][0], path[-1][1] - path[-2][1])
                    )
                else:
                    direction = unit_vector(
                        (path[-1][0] - start[0], path[-1][1] - start[1])
                    )

                remaining = PATH_LENGTH - current_length
                last = path[-1]

                while remaining > 1e-9:
                    step = min(STEP, remaining)

                    # Never exceed the 10 m maximum.
                    if distance(start, last) + step > MAX_PATH_LENGTH:
                        break

                    last = (last[0] + direction[0] * step, last[1] + direction[1] * step)
                    path.append(last)
                    remaining -= step

        return path