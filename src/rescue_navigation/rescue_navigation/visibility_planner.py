import math
import cv2
import numpy as np

from .map_loader import grid_to_world


def plan_scan_points(
    grid,
    labels,
    component,
    resolution,
    origin,
    height,
    robot_name,
    split_y=0.0,
    lower_robot='tb3_0'
):
    """
    Velger skannepunkter basert på synlige vegger.

    Bruker kartet til å finne steder hvor roboten
    kan observere mange veggflater fra kort avstand.
    """

    # Maksimal avstand vi undersøker fra hvert punkt
    max_view_distance = 2.0

    # Avstand mellom mulige skanneposisjoner
    candidate_spacing = 0.8

    # Maksimalt antall skannepunkter per robot
    max_scan_points = 12

    # Antall retninger kameraet kan undersøke
    num_rays = 36

    # Beregn avstand til hindringer
    free_space = (grid == 0).astype(np.uint8)

    distance_map = cv2.distanceTransform(
        free_space,
        cv2.DIST_L2,
        5
    )

    step = max(
        1,
        round(candidate_spacing / resolution)
    )

    max_steps = max(
        1,
        round(max_view_distance / resolution)
    )

    # Samle mulige skannepunkter
    candidates = []

    for r in range(0, height, step):
        for c in range(0, grid.shape[1], step):

            if labels[r, c] != component:
                continue

            distance_to_wall = (
                distance_map[r, c] * resolution
            )

            # Ikke helt inntil veggen,
            # men heller ikke for langt unna
            if not 0.35 <= distance_to_wall <= 1.8:
                continue

            x, y = grid_to_world(
                r,
                c,
                resolution,
                origin,
                height
            )

            # Fordel bygningen mellom robotene
            if robot_name == lower_robot and y >= split_y:
                continue

            if robot_name != lower_robot and y < split_y:
                continue

            candidates.append((r, c, x, y))

    # Beregn hvilke veggflater som er synlige
    visibility = []

    angles = np.linspace(
        0,
        2 * math.pi,
        num_rays,
        endpoint=False
    )

    for r, c, x, y in candidates:

        visible_walls = set()

        for angle in angles:

            dr = math.sin(angle)
            dc = math.cos(angle)

            for distance in range(1, max_steps + 1):

                rr = round(r + dr * distance)
                cc = round(c + dc * distance)

                if not (
                    0 <= rr < height
                    and 0 <= cc < grid.shape[1]
                ):
                    break

                # Første hindring stopper siktlinjen
                if grid[rr, cc] != 0:

                    # Grupper nærliggende veggceller
                    # slik at vi ikke teller samme
                    # vegg mange ganger
                    wall = (
                        rr // 4,
                        cc // 4
                    )

                    visible_walls.add(wall)
                    break

        visibility.append(visible_walls)

    # Velg punkter som samlet dekker flest vegger
    selected = []
    covered_walls = set()
    remaining_indices = set(range(len(candidates)))

    while remaining_indices and len(selected) < max_scan_points:

        best_index = max(
            remaining_indices,
            key=lambda i: len(
                visibility[i] - covered_walls
            )
        )

        new_walls = visibility[best_index] - covered_walls

        if not new_walls:
            break

        selected.append(
            (
                candidates[best_index][2],
                candidates[best_index][3]
            )
        )

        covered_walls.update(
            visibility[best_index]
        )

        remaining_indices.remove(best_index)

    return selected