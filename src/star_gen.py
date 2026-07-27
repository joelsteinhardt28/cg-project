import math
import argparse

def generate_star_off(num_arms, filename, r_in=1.0, r_out=3.0, height=1.5):
    """
    Generates a 3D star with `num_arms` and saves it as an OFF file.
    """
    # 2 points per arm on the equator (one inner, one outer)
    num_equator_points = 2 * num_arms
    
    # Total vertices: Equator points + 1 Top Pole + 1 Bottom Pole
    num_vertices = num_equator_points + 2
    
    # Total faces: 2 triangles per equator segment (one top, one bottom)
    num_faces = 2 * num_equator_points

    vertices = []
    
    # 0: Top pole
    vertices.append((0.0, 0.0, height))
    # 1: Bottom pole
    vertices.append((0.0, 0.0, -height))

    # Equator points (alternating outer tips and inner valleys)
    for i in range(num_equator_points):
        # Calculate angle for this point
        angle = i * math.pi / num_arms
        
        # Even indices are outer points (tips), odd indices are inner points (valleys)
        radius = r_out if i % 2 == 0 else r_in
        
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        vertices.append((x, y, 0.0))

    faces = []
    # Generate the triangular faces
    for i in range(num_equator_points):
        current_eq = 2 + i
        next_eq = 2 + ((i + 1) % num_equator_points)
        
        # Top face (connected to Top Pole at index 0)
        # Winding order is counter-clockwise to ensure the normal points outward
        faces.append((3, 0, current_eq, next_eq))
        
        # Bottom face (connected to Bottom Pole at index 1)
        # Winding order is reversed to ensure the normal points outward
        faces.append((3, 1, next_eq, current_eq))

    # Write data to the OFF file
    with open(filename, 'w') as f:
        f.write("OFF\n")
        # Write header: Vertices, Faces, Edges (Edges can be 0 in OFF files)
        f.write(f"{num_vertices} {num_faces} 0\n")
        
        # Write all vertices
        for v in vertices:
            f.write(f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
            
        # Write all faces
        for face in faces:
            f.write(f"{face[0]} {face[1]} {face[2]} {face[3]}\n")
            
    print(f"Successfully generated {filename} with {num_arms} arms.")
    print(f" -> Vertices: {num_vertices}")
    print(f" -> Faces: {num_faces}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a 3D star OFF file.")
    parser.add_argument("-n", "--arms", type=int, default=256, help="Number of arms for the star")
    parser.add_argument("-o", "--output", type=str, default="star.off", help="Output filename")
    
    args = parser.parse_args()
    generate_star_off(args.arms, args.output)