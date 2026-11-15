from astar import astar


grid = [
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0]
]

start = (0, 0)
goal = (5, 5)

path = astar(grid, start, goal)

print("Funnet rute:", path)

if path:
    for row, col in path:
        if (row, col) != start and (row, col) != goal:
            grid[row][col] = 2

    for row in grid:
        print(" ".join(str(cell) for cell in row))