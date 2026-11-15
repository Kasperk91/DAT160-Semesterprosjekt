import numpy as np
import cv2
import yaml


def load_map(yaml_path, safety_margin=0.20):

    # Les informasjon fra YAML-filen
    with open(yaml_path, 'r') as file:
        map_info = yaml.safe_load(file)

    resolution = map_info['resolution']
    origin = map_info['origin']

    # Finn kartbildet i samme mappe som YAML-filen
    import os
    image_path = os.path.join(
        os.path.dirname(yaml_path),
        map_info['image']
    )

    # Les bildet i gråtoner
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    if image is None:
        raise FileNotFoundError(f"Fant ikke kartbildet: {image_path}")

    # Svart = hindring (1), hvitt = ledig (0)
    grid = np.where(image < 128, 1, 0).astype(np.uint8)

    # Utvid hindringene med sikkerhetsmargin
    margin_pixels = int(safety_margin / resolution)

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (margin_pixels * 2 + 1, margin_pixels * 2 + 1)
    )

    grid = cv2.dilate(grid, kernel)

    return grid, resolution, origin


def world_to_grid(x, y, resolution, origin, map_height):

    col = int((x - origin[0]) / resolution)
    row = map_height - 1 - int((y - origin[1]) / resolution)

    return row, col


def grid_to_world(row, col, resolution, origin, map_height):

    x = origin[0] + (col + 0.5) * resolution
    y = origin[1] + (map_height - row - 0.5) * resolution

    return x, y