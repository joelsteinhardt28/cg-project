import math
import argparse

def write_off(filename, vertices, faces, integer_scale=None):
    """Writes the vertices and faces to an OFF file."""
    with open(filename, 'w') as f:
        f.write("OFF\n")
        f.write(f"{len(vertices)} {len(faces)} 0\n")
        
        for v in vertices:
            if integer_scale:
                f.write(f"{int(v[0] * integer_scale)} {int(v[1] * integer_scale)} {int(v[2] * integer_scale)}\n")
            else:
                f.write(f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
                
        for face in faces:
            f.write(f"3 {face[0]} {face[1]} {face[2]}\n")
            
    print(f"Generated {filename}: {len(vertices)} vertices, {len(faces)} faces.")

def generate_pacman(resolution, filename, integer_scale=None):
    """
    Generates a spherical 'Pac-Man' (a sphere with a wedge removed).
    resolution dictates the number of subdivisions.
    """
    radius = 5.0
    # The wedge opening angle (e.g., 90 degrees removed)
    cutout_angle = math.pi / 2.0 
    
    steps_phi = resolution      # Vertical (latitude) steps
    steps_theta = resolution    # Horizontal (longitude) steps
    
    vertices = []
    
    # Core vertices for the cutout
    vertices.append((0.0, 0.0, 0.0))  # 0: Origin (Center of the mouth)
    vertices.append((0.0, 0.0, radius)) # 1: North Pole
    vertices.append((0.0, 0.0, -radius)) # 2: South Pole

    # Generate grid vertices
    grid_start_idx = len(vertices)
    
    for i in range(1, steps_phi):
        phi = math.pi * i / steps_phi
        z = radius * math.cos(phi)
        r_xy = radius * math.sin(phi)
        
        for j in range(steps_theta + 1):
            # Map j from 0 to steps_theta onto the range [cutout_angle/2, 2*pi - cutout_angle/2]
            theta = (cutout_angle / 2.0) + j * ((2.0 * math.pi - cutout_angle) / steps_theta)
            
            x = r_xy * math.cos(theta)
            y = r_xy * math.sin(theta)
            vertices.append((x, y, z))

    faces = []
    
    def get_idx(i, j):
        return grid_start_idx + (i - 1) * (steps_theta + 1) + j

    # 1. Sphere Surface Faces
    for j in range(steps_theta):
        # Top cap (connected to North Pole)
        faces.append((1, get_idx(1, j), get_idx(1, j + 1)))
        
        # Bottom cap (connected to South Pole)
        faces.append((2, get_idx(steps_phi - 1, j + 1), get_idx(steps_phi - 1, j)))

    # Mid-body quads split into triangles
    for i in range(1, steps_phi - 1):
        for j in range(steps_theta):
            v0 = get_idx(i, j)
            v1 = get_idx(i, j + 1)
            v2 = get_idx(i + 1, j + 1)
            v3 = get_idx(i + 1, j)
            
            faces.append((v0, v3, v1))
            faces.append((v1, v3, v2))

    # 2. Cutout Caps (The "Mouth")
    # Connect the origin to the boundary edges of the wedge
    for i in range(1, steps_phi):
        # Top half connected to poles handled via origin
        if i == 1:
            faces.append((0, get_idx(i, 0), 1))
            faces.append((0, 1, get_idx(i, steps_theta)))
        if i == steps_phi - 1:
            faces.append((0, 2, get_idx(i, 0)))
            faces.append((0, get_idx(i, steps_theta), 2))
            
        if i < steps_phi - 1:
            # Face 1 of mouth (j = 0)
            faces.append((0, get_idx(i + 1, 0), get_idx(i, 0)))
            # Face 2 of mouth (j = steps_theta)
            faces.append((0, get_idx(i, steps_theta), get_idx(i + 1, steps_theta)))

    write_off(filename, vertices, faces, integer_scale)

def generate_hourglass(resolution, filename, integer_scale=None):
    """
    Generates a pinched tube (hourglass) shape.
    """
    steps_z = resolution
    steps_theta = resolution
    
    height = 10.0
    
    vertices = []
    vertices.append((0.0, 0.0, height))   # 0: Top Pole
    vertices.append((0.0, 0.0, -height))  # 1: Bottom Pole

    grid_start_idx = len(vertices)

    # Generate body vertices
    for i in range(steps_z + 1):
        # Map i to z range [-height, height]
        t = i / steps_z
        z = -height + 2.0 * height * t
        
        # Radius function creates the pinch at z=0 using a cosine curve
        # R(z) = 4.0 - 2.5 * cos(pi * z / 2*height)
        r = 4.0 - 2.5 * math.cos(math.pi * z / (2.0 * height))
        
        for j in range(steps_theta):
            theta = j * 2.0 * math.pi / steps_theta
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            vertices.append((x, y, z))

    faces = []

    def get_idx(i, j):
        return grid_start_idx + i * steps_theta + (j % steps_theta)

    # Caps
    for j in range(steps_theta):
        # Bottom Cap
        faces.append((1, get_idx(0, j + 1), get_idx(0, j)))
        # Top Cap
        faces.append((0, get_idx(steps_z, j), get_idx(steps_z, j + 1)))

    # Mid-body quads split into triangles
    for i in range(steps_z):
        for j in range(steps_theta):
            v0 = get_idx(i, j)
            v1 = get_idx(i, j + 1)
            v2 = get_idx(i + 1, j + 1)
            v3 = get_idx(i + 1, j)
            
            faces.append((v0, v1, v3))
            faces.append((v1, v2, v3))

    write_off(filename, vertices, faces, integer_scale)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate stress-test OFF files for mesh kernels.")
    parser.add_argument("shape", choices=['pacman', 'hourglass'], help="Shape to generate")
    parser.add_argument("-r", "--resolution", type=int, default=32, help="Subdivision resolution (higher = more faces)")
    parser.add_argument("-o", "--output", type=str, default="test_mesh.off", help="Output filename")
    parser.add_argument("--int", dest="integer_scale", type=int, default=None, 
                        help="Scale floats by this multiplier and cast to integers for exact math (e.g., 10000)")
    
    args = parser.parse_args()
    
    if args.shape == 'pacman':
        generate_pacman(args.resolution, args.output, args.integer_scale)
    elif args.shape == 'hourglass':
        generate_hourglass(args.resolution, args.output, args.integer_scale)