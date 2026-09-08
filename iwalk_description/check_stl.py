#!/usr/bin/env python3

import struct
import sys
from pathlib import Path


def binary_vertices(data):
    if len(data) < 84:
        return None

    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50

    if expected_size > len(data):
        return None

    vertices = []
    offset = 84

    for _ in range(triangle_count):
        # 12 bytes normal vector
        offset += 12

        for _ in range(3):
            vertex = struct.unpack_from("<fff", data, offset)
            vertices.append(vertex)
            offset += 12

        # 2-byte attribute count
        offset += 2

    return vertices


def ascii_vertices(data):
    try:
        text = data.decode("utf-8", errors="strict")
    except UnicodeDecodeError:
        return None

    vertices = []

    for line in text.splitlines():
        parts = line.strip().split()

        if len(parts) == 4 and parts[0].lower() == "vertex":
            try:
                vertices.append(
                    tuple(float(value) for value in parts[1:4])
                )
            except ValueError:
                pass

    return vertices if vertices else None


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 check_stl.py path/to/model.stl")
        raise SystemExit(1)

    path = Path(sys.argv[1])
    data = path.read_bytes()

    vertices = binary_vertices(data)
    stl_type = "binary"

    if not vertices:
        vertices = ascii_vertices(data)
        stl_type = "ASCII"

    if not vertices:
        print("Could not extract STL vertices.")
        raise SystemExit(2)

    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    zs = [v[2] for v in vertices]

    minimum = (min(xs), min(ys), min(zs))
    maximum = (max(xs), max(ys), max(zs))

    dimensions = tuple(
        maximum[i] - minimum[i]
        for i in range(3)
    )

    center = tuple(
        (minimum[i] + maximum[i]) / 2.0
        for i in range(3)
    )

    print(f"File:       {path}")
    print(f"STL type:   {stl_type}")
    print(f"Vertices:   {len(vertices)}")
    print()
    print(f"Minimum:    x={minimum[0]:.6f}, "
          f"y={minimum[1]:.6f}, "
          f"z={minimum[2]:.6f}")

    print(f"Maximum:    x={maximum[0]:.6f}, "
          f"y={maximum[1]:.6f}, "
          f"z={maximum[2]:.6f}")

    print(f"Dimensions: x={dimensions[0]:.6f}, "
          f"y={dimensions[1]:.6f}, "
          f"z={dimensions[2]:.6f}")

    print(f"Box center: x={center[0]:.6f}, "
          f"y={center[1]:.6f}, "
          f"z={center[2]:.6f}")


if __name__ == "__main__":
    main()
