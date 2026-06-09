import trimesh

# Load your wheel mesh
mesh = trimesh.load('LF.STL')

# Get the bounding box extents (X, Y, Z)
extents = mesh.extents
print(f"Bounding Box (X, Y, Z): {extents}")

# Assuming the wheel is a cylinder, the thickness is the smallest dimension
thickness = min(extents)

# The diameter is the largest dimension
diameter = max(extents)
radius = diameter / 2

print(f"URDF Cylinder length: {thickness:.4f}")
print(f"URDF Cylinder radius: {radius:.4f}")