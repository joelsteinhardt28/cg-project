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


def generate_hourglass(resolution, filename, integer_scale=None):
    steps_z = resolution
    steps_theta = resolution
    height = 10.0
    
    vertices = []
    vertices.append((0.0, 0.0, height))
    vertices.append((0.0, 0.0, -height))
    grid_start_idx = len(vertices)

    for i in range(steps_z + 1):
        t = i / steps_z
        z = -height + 2.0 * height * t
        r = 4.0 - 2.5 * math.cos(math.pi * z / (2.0 * height))
        
        for j in range(steps_theta):
            theta = j * 2.0 * math.pi / steps_theta
            vertices.append((r * math.cos(theta), r * math.sin(theta), z))

    faces = []
    def get_idx(i, j): return grid_start_idx + i * steps_theta + (j % steps_theta)

    for j in range(steps_theta):
        faces.append((1, get_idx(0, j + 1), get_idx(0, j)))
        faces.append((0, get_idx(steps_z, j), get_idx(steps_z, j + 1)))

    for i in range(steps_z):
        for j in range(steps_theta):
            v0, v1 = get_idx(i, j), get_idx(i, j + 1)
            v2, v3 = get_idx(i + 1, j + 1), get_idx(i + 1, j)
            faces.extend([(v0, v1, v3), (v1, v2, v3)])

    write_off(filename, vertices, faces, integer_scale)


def generate_gear_cylinder(resolution, filename, integer_scale=None):
    """Generates an extruded 2D star/gear.
       Guaranteed non-empty kernel (a central cylinder)."""
    steps_z = resolution
    steps_theta = resolution * 2
    height = 5.0
    
    vertices = [(0.0, 0.0, height), (0.0, 0.0, -height)]
    grid_start_idx = len(vertices)

    for i in range(steps_z + 1):
        t = i / steps_z
        z = -height + 2.0 * height * t
        
        for j in range(steps_theta):
            theta = j * 2.0 * math.pi / steps_theta
            # Radius oscillates to create gear teeth but leaves a solid core
            r = 4.0 + 1.5 * math.cos(8.0 * theta)
            vertices.append((r * math.cos(theta), r * math.sin(theta), z))

    faces = []
    def get_idx(i, j): return grid_start_idx + i * steps_theta + (j % steps_theta)

    for j in range(steps_theta):
        faces.append((1, get_idx(0, j + 1), get_idx(0, j)))
        faces.append((0, get_idx(steps_z, j), get_idx(steps_z, j + 1)))

    for i in range(steps_z):
        for j in range(steps_theta):
            v0, v1 = get_idx(i, j), get_idx(i, j + 1)
            v2, v3 = get_idx(i + 1, j + 1), get_idx(i + 1, j)
            faces.extend([(v0, v1, v3), (v1, v2, v3)])

    write_off(filename, vertices, faces, integer_scale)


def generate_wheel(resolution, filename, integer_scale=None):
    """Generates a tube with a large central bulge that tapers concavely.
       Guaranteed to have a non-empty kernel at the center."""
    steps_z = resolution * 2
    steps_theta = resolution
    max_z = 8.0
    
    vertices = [(0.0, 0.0, max_z), (0.0, 0.0, -max_z)]
    grid_start_idx = len(vertices)

    for i in range(steps_z + 1):
        t = i / steps_z
        z = -max_z + 2.0 * max_z * t
        
        # Gaussian bump in the center, tapering concavely to a cylinder
        r = 1.5 + 4.0 * math.exp(-(z**2) / 4.0)
        
        for j in range(steps_theta):
            theta = j * 2.0 * math.pi / steps_theta
            vertices.append((r * math.cos(theta), r * math.sin(theta), z))

    faces = []
    def get_idx(i, j): return grid_start_idx + i * steps_theta + (j % steps_theta)

    for j in range(steps_theta):
        faces.append((1, get_idx(0, j + 1), get_idx(0, j)))
        faces.append((0, get_idx(steps_z, j), get_idx(steps_z, j + 1)))

    for i in range(steps_z):
        for j in range(steps_theta):
            v0, v1 = get_idx(i, j), get_idx(i, j + 1)
            v2, v3 = get_idx(i + 1, j + 1), get_idx(i + 1, j)
            faces.extend([(v0, v1, v3), (v1, v2, v3)])

    write_off(filename, vertices, faces, integer_scale)


def generate_shallow_spiky_sphere(resolution, filename, integer_scale=None):
    """Generates a sphere with shallow omni-directional dimples.
       Guaranteed non-empty kernel."""
    steps_phi = resolution
    steps_theta = resolution * 2
    
    R = 5.0
    A = 0.8  # Shallow amplitude prevents self-occlusion of the origin
    freq_phi = 4.0
    freq_theta = 6.0
    
    vertices = [(0.0, 0.0, R), (0.0, 0.0, -R)]
    grid_start_idx = len(vertices)

    for i in range(1, steps_phi):
        phi = math.pi * i / steps_phi
        for j in range(steps_theta):
            theta = j * 2.0 * math.pi / steps_theta
            r = R + A * math.sin(freq_phi * phi) * math.cos(freq_theta * theta)
            
            x = r * math.sin(phi) * math.cos(theta)
            y = r * math.sin(phi) * math.sin(theta)
            z = r * math.cos(phi)
            vertices.append((x, y, z))

    faces = []
    def get_idx(i, j): return grid_start_idx + (i - 1) * steps_theta + (j % steps_theta)

    for j in range(steps_theta):
        faces.append((0, get_idx(1, j), get_idx(1, j + 1)))
        faces.append((1, get_idx(steps_phi - 1, j + 1), get_idx(steps_phi - 1, j)))

    for i in range(1, steps_phi - 1):
        for j in range(steps_theta):
            v0, v1 = get_idx(i, j), get_idx(i, j + 1)
            v2, v3 = get_idx(i + 1, j + 1), get_idx(i + 1, j)
            faces.extend([(v0, v3, v1), (v1, v3, v2)])

    write_off(filename, vertices, faces, integer_scale)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate stress-test OFF files for mesh kernels.")
    parser.add_argument("shape", choices=['hourglass', 'gear', 'wheel', 'wad'], help="Shape to generate")
    parser.add_argument("-r", "--resolution", type=int, default=32, help="Subdivision resolution (higher = more faces)")
    parser.add_argument("-o", "--output", type=str, default="test_mesh.off", help="Output filename")
    parser.add_argument("--int", dest="integer_scale", type=int, default=None, 
                        help="Scale floats by this multiplier and cast to integers for exact math (e.g., 10000)")
    
    args = parser.parse_args()
    
    if args.shape == 'hourglass':
        generate_hourglass(args.resolution, args.output, args.integer_scale)
    elif args.shape == 'gear':
        generate_gear_cylinder(args.resolution, args.output, args.integer_scale)
    elif args.shape == 'wheel':
        generate_wheel(args.resolution, args.output, args.integer_scale)
    elif args.shape == 'wad':
        generate_shallow_spiky_sphere(args.resolution, args.output, args.integer_scale)