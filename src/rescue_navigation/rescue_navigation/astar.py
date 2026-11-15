import heapq
import math


def heuristic(a, b):
    """Beregner avstanden i rett linje mellom to punkter."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def astar(grid, start, goal):
    """
    Finner korteste rute fra start til mål med A*.

    grid: 2D-liste der 0 = fri plass og 1 = hindring.
    start: (rad, kolonne)
    goal: (rad, kolonne)

    Returnerer en liste med punkter, eller None hvis ingen rute finnes.
    """

    rows = len(grid)
    cols = len(grid[0])

    # Kontroller at start og mål er innenfor kartet
    for row, col in (start, goal):
        if not (0 <= row < rows and 0 <= col < cols):
            return None

        if grid[row][col] != 0:
            return None

    # Mulige bevegelser: opp, ned, venstre, høyre og diagonalt
    directions = [
        (-1, 0), (1, 0),
        (0, -1), (0, 1),
        (-1, -1), (-1, 1),
        (1, -1), (1, 1)
    ]

    # Prioritetskø: velger punktet med lavest estimert totalkostnad
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), start))

    came_from = {}
    g_score = {start: 0}

    while open_set:

        _, current = heapq.heappop(open_set)

        # Målet er funnet
        if current == goal:

            path = [current]

            while current in came_from:
                current = came_from[current]
                path.append(current)

            path.reverse()
            return path

        for dr, dc in directions:

            nr = current[0] + dr
            nc = current[1] + dc

            # Sjekk kartgrensene
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue

            # Ikke kjør gjennom vegger
            if grid[nr][nc] != 0:
                continue

            # Unngå å kutte diagonalt gjennom vegghjørner
            if dr != 0 and dc != 0:
                if (
                    grid[current[0] + dr][current[1]] != 0
                    or grid[current[0]][current[1] + dc] != 0
                ):
                    continue

            neighbor = (nr, nc)

            # Diagonal bevegelse er lengre enn rett bevegelse
            movement_cost = math.hypot(dr, dc)
            tentative_g = g_score[current] + movement_cost

            if tentative_g < g_score.get(neighbor, float('inf')):

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g

                f_score = tentative_g + heuristic(neighbor, goal)

                heapq.heappush(open_set, (f_score, neighbor))

    return None