import os
import cv2

from astar import astar
from map_loader import load_map, world_to_grid

# Finn konkurransekartet
map_path = os.path.expanduser(
    "~/ros2_ws/src/multi_robot_challenge_23/maps/map_dat160_w1.yaml"
)

# Les kartet med 20 cm sikkerhetsmargin
grid, resolution, origin = load_map(map_path, safety_margin=0.20)

height, width = grid.shape

print(f"Kartstørrelse: {width} x {height} piksler")

# Robot 1 sin startposisjon
start_x = 0.0
start_y = -1.0

# Foreløpig testmål, ikke en hardkodet konkurranserute
goal_x = 0.0
goal_y = 3.0

# Konverter ROS-koordinater til kartceller
start = world_to_grid(start_x, start_y, resolution, origin, height)
goal = world_to_grid(goal_x, goal_y, resolution, origin, height)

print(f"Startcelle: {start}")
print(f"målcelle: {goal}")

print(f"Start blokkert: {grid[start] != 0}")
print(f"Mål blokkert: {grid[goal] != 0}")

# Planlegg ruten
path = astar(grid, start, goal)

if path is None:
    print("Fant ingen rute. Start eller mål kan være blokkert.")

else:
    print(f"Fant en rute med {len(path)} punkter!")

    # Les originalbildet for visualisering
    image_path = map_path.replace(".yaml", ".png")
    image = cv2.imread(image_path)

    # Tegn ruten på kartet
    for row, col in path:
        cv2.circle(image, (col, row), 2, (0, 0, 255), -1)

    # Marker start og mål
    cv2.circle(image, (start[1], start[0]), 12, (0, 255, 0), -1)
    cv2.circle(image, (goal[1], goal[0]), 12, (255, 0, 0), -1)

    output_path = os.path.expanduser("~/astar_result.png")
    cv2.imwrite(output_path, image)

    print(f"Kart med rute lagret som: {output_path}")