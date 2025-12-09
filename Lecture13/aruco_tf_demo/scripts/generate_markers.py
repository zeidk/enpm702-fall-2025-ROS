#!/usr/bin/env python3
"""
Generate ArUco marker images for the aruco_tf_demo package.

This script generates PNG images for ArUco markers and saves them
to the appropriate materials/textures directories.

Requirements:
    pip install opencv-python opencv-contrib-python

Usage:
    python3 generate_markers.py

    # Or specify custom parameters:
    python3 generate_markers.py --size 300 --dictionary DICT_5X5_50 --ids 0 1 2 3 4
"""

import argparse
import os
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    print("Error: OpenCV is required. Install with:")
    print("  pip install opencv-python opencv-contrib-python")
    sys.exit(1)


# Dictionary name to OpenCV constant mapping
DICTIONARY_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def generate_marker(dictionary, marker_id: int, size: int, border_bits: int = 1) -> np.ndarray:
    """
    Generate an ArUco marker image.
    
    Args:
        dictionary: OpenCV ArUco dictionary
        marker_id: ID of the marker to generate
        size: Size of the marker image in pixels
        border_bits: Width of the white border in bits
        
    Returns:
        numpy array containing the marker image
    """
    marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, size)
    return marker_img


def add_white_border(image: np.ndarray, border_pixels: int) -> np.ndarray:
    """
    Add a white border around the marker image.
    
    Args:
        image: Marker image
        border_pixels: Width of border in pixels
        
    Returns:
        Image with white border
    """
    return cv2.copyMakeBorder(
        image,
        border_pixels, border_pixels, border_pixels, border_pixels,
        cv2.BORDER_CONSTANT,
        value=255
    )


def main():
    parser = argparse.ArgumentParser(
        description="Generate ArUco marker images for Gazebo simulation"
    )
    parser.add_argument(
        "--size", type=int, default=200,
        help="Marker image size in pixels (default: 200)"
    )
    parser.add_argument(
        "--border", type=int, default=20,
        help="White border width in pixels (default: 20)"
    )
    parser.add_argument(
        "--dictionary", type=str, default="DICT_4X4_50",
        choices=list(DICTIONARY_MAP.keys()),
        help="ArUco dictionary to use (default: DICT_4X4_50)"
    )
    parser.add_argument(
        "--ids", type=int, nargs="+", default=[0, 1, 2],
        help="Marker IDs to generate (default: 0 1 2)"
    )
    parser.add_argument(
        "--output-dir", type=str, default=None,
        help="Output directory (default: auto-detect package models directory)"
    )
    
    args = parser.parse_args()
    
    # Get ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICTIONARY_MAP[args.dictionary])
    
    # Determine output directory
    if args.output_dir:
        base_dir = args.output_dir
    else:
        # Try to find the package models directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_dir = os.path.join(script_dir, "models")
        
        if not os.path.exists(base_dir):
            # Fallback to current directory
            base_dir = os.getcwd()
            print(f"Warning: models directory not found, using current directory: {base_dir}")
    
    print("Generating ArUco markers:")
    print(f"  Dictionary: {args.dictionary}")
    print(f"  Size: {args.size}x{args.size} pixels")
    print(f"  Border: {args.border} pixels")
    print(f"  IDs: {args.ids}")
    print()
    
    for marker_id in args.ids:
        # Generate marker
        marker_img = generate_marker(aruco_dict, marker_id, args.size)
        
        # Add white border
        marker_with_border = add_white_border(marker_img, args.border)
        
        # Determine output path
        model_dir = os.path.join(base_dir, f"aruco_marker_{marker_id}")
        textures_dir = os.path.join(model_dir, "materials", "textures")
        
        # Create directories if they don't exist
        os.makedirs(textures_dir, exist_ok=True)
        
        # Save image
        output_path = os.path.join(textures_dir, f"aruco_marker_{marker_id}.png")
        cv2.imwrite(output_path, marker_with_border)
        
        print(f"  Generated marker {marker_id}: {output_path}")
    
    print()
    print("Done! Now update the model.sdf files to use the textures:")
    print("  1. Edit each model.sdf file")
    print("  2. Uncomment the <script> or <pbr> block in the material section")
    print("  3. Comment out or remove the placeholder <ambient>/<diffuse> colors")


if __name__ == "__main__":
    main()
