#!/usr/bin/env python3

import sys
import yaml
import numpy as np
import cv2
import matplotlib.pyplot as plt


def load_yaml(yaml_path):
    """Load map metadata from YAML file."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data


def load_pgm(pgm_path):
    """Load PGM image as grayscale."""
    img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)

    if img is None:
        raise RuntimeError(f"Failed to load image: {pgm_path}")

    return img


def visualize_map(img, yaml_data):
    resolution = yaml_data["resolution"]
    origin = yaml_data["origin"]

    height, width = img.shape

    # Correct approach: fix image once
    img_corrected = np.flipud(img)

    # Map bounds in world coordinates
    x_min = origin[0]
    x_max = origin[0] + width * resolution
    y_min = origin[1]
    y_max = origin[1] + height * resolution

    plt.figure(figsize=(8, 6))
    plt.imshow(
        img_corrected,
        cmap='gray',
        extent=[x_min, x_max, y_min, y_max],
        origin='lower'
    )
    plt.title("Map Visualization (Corrected to ROS Frame)")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis("equal")
    plt.colorbar(label="Pixel value")

    print(f"Map size (pixels): {width} x {height}")
    print(f"Resolution: {resolution} m/pixel")
    print(f"Origin: {origin}")

    output_path = "map_visualization.png"
    plt.savefig(output_path, dpi=150)
    print(f"Saved visualization to {output_path}")


def main():
    if len(sys.argv) != 3:
        print("Usage: python generate_centerline.py <map.pgm> <map.yaml>")
        sys.exit(1)

    pgm_path = sys.argv[1]
    yaml_path = sys.argv[2]

    # Load data
    yaml_data = load_yaml(yaml_path)
    img = load_pgm(pgm_path)

    # Visualize
    visualize_map(img, yaml_data)


if __name__ == "__main__":
    main()