tool
extends Spatial

export(bool) var update = false setget set_update

# Function to check if a point is inside the convex shape
func is_point_inside_convex_shape(point: Vector3, hull_faces: Array) -> bool:
	for face in hull_faces:
		var a = face[0]
		var b = face[1]
		var c = face[2]

		# Compute the plane equation (normal and d)
		var normal = (b - a).cross(c - a).normalized()
		var d = normal.dot(a)

		# Check if the point is on the correct side of the plane
		if normal.dot(point) > d:
			return false  # Point is outside this face

	return true  # Point is inside or on all faces

# Helper function to compute the distance from a point to a plane
func distance_to_plane(point: Vector3, plane: Array) -> float:
	var normal = plane[0]
	var d = plane[1]
	return normal.dot(point) - d

# Helper function to compute the plane equation from three points
func plane_from_points(a: Vector3, b: Vector3, c: Vector3) -> Array:
	var normal = (b - a).cross(c - a).normalized()
	var d = normal.dot(a)
	return [normal, d]

# Helper function to check if four points are coplanar
func are_points_coplanar(a: Vector3, b: Vector3, c: Vector3, d: Vector3) -> bool:
	var normal = (b - a).cross(c - a).normalized()
	var distance = normal.dot(d - a)
	return abs(distance) < 1e-6  # Allow for a small floating-point error

# Recursive function to build the convex hull with quads
func quickhull(points: PoolVector3Array, a: Vector3, b: Vector3, c: Vector3, d: Vector3, hull_faces: Array, depth: int = 0):
	if depth > 100:  # Add a maximum recursion depth to prevent infinite loops
		print("Recursion depth exceeded!")
		return

	var plane = plane_from_points(a, b, c)
	var farthest_point = null
	var max_distance = 0.0

	# Find the farthest point from the plane
	for p in points:
		var distance = abs(distance_to_plane(p, plane))
		if distance > max_distance:
			max_distance = distance
			farthest_point = p

	if farthest_point == null:
		# No more points to add, this is a face of the hull
		if are_points_coplanar(a, b, c, d):  # Ensure the quad is coplanar
			hull_faces.append([a, b, c, d])  # Store quad vertices
		return

	# Split the points into four new regions
	var new_points = PoolVector3Array()
	for p in points:
		if p != farthest_point:
			new_points.append(p)

	# Recursively build the hull for the new regions
	quickhull(new_points, a, b, farthest_point, d, hull_faces, depth + 1)
	quickhull(new_points, b, c, farthest_point, d, hull_faces, depth + 1)
	quickhull(new_points, c, d, farthest_point, d, hull_faces, depth + 1)
	quickhull(new_points, d, a, farthest_point, d, hull_faces, depth + 1)

# Main function to compute the convex hull with quads
func convex_hull_3d(points: PoolVector3Array) -> Array:
	if points.size() < 4:
		return []  # Not enough points to form a 3D hull

	# Find the initial tetrahedron
	var min_x = points[0]
	var max_x = points[0]
	var min_y = points[0]
	var max_y = points[0]
	var min_z = points[0]
	var max_z = points[0]

	for p in points:
		if p.x < min_x.x: min_x = p
		if p.x > max_x.x: max_x = p
		if p.y < min_y.y: min_y = p
		if p.y > max_y.y: max_y = p
		if p.z < min_z.z: min_z = p
		if p.z > max_z.z: max_z = p

	var hull_faces = []
	quickhull(points, min_x, max_x, min_y, max_y, hull_faces)
	quickhull(points, max_x, max_y, min_z, max_z, hull_faces)
	quickhull(points, max_y, min_y, max_z, min_z, hull_faces)
	quickhull(points, min_y, min_z, max_z, min_x, hull_faces)

	# Fix face orientation
	hull_faces = fix_face_orientation(hull_faces)

	return hull_faces

# Function to fix face orientation (ensure normals point outward)
func fix_face_orientation(hull_faces: Array) -> Array:
	var fixed_faces = []

	# Calculate the centroid of the convex hull
	var centroid = Vector3.ZERO
	for face in hull_faces:
		centroid += face[0] + face[1] + face[2] + face[3]
	centroid /= hull_faces.size() * 4

	for face in hull_faces:
		var a = face[0]
		var b = face[1]
		var c = face[2]
		var d = face[3]

		# Calculate the normal of the face
		var normal = (b - a).cross(c - a).normalized()

		# Calculate the vector from the centroid to the face's centroid
		var face_centroid = (a + b + c + d) / 4.0
		var centroid_to_face = face_centroid - centroid

		# If the normal points inward, flip the winding order
		if normal.dot(centroid_to_face) < 0:
			fixed_faces.append([a, d, c, b])  # Flip winding order
		else:
			fixed_faces.append([a, b, c, d])  # Keep winding order

	return fixed_faces

# Function to filter out duplicate faces
func filter_duplicate_faces(hull_faces: Array) -> Array:
	var unique_faces = []
	var seen_faces = {}

	for face in hull_faces:
		# Sort the vertices of the face to ensure consistent comparison
		var sorted_face = [face[0], face[1], face[2], face[3]]
		sorted_face.sort()

		# Create a unique key for the face
		var key = "%s-%s-%s-%s" % [sorted_face[0], sorted_face[1], sorted_face[2], sorted_face[3]]

		# Add the face to the unique list if it hasn't been seen before
		if not seen_faces.has(key):
			seen_faces[key] = true
			unique_faces.append(face)

	return unique_faces

# Function to filter out degenerate faces
func filter_degenerate_faces(hull_faces: Array) -> Array:
	var valid_faces = []

	for face in hull_faces:
		var a = face[0]
		var b = face[1]
		var c = face[2]
		var d = face[3]

		# Calculate the area of the quad
		var edge1 = b - a
		var edge2 = c - a
		var normal = edge1.cross(edge2)
		var area = normal.length()

		# Keep the face if it has a non-zero area
		if area > 1e-6:
			valid_faces.append(face)

	return valid_faces

# Function to check if a ray intersects with any face in the hull
func ray_intersects_hull(origin: Vector3, direction: Vector3, hull_faces: Array) -> bool:
	for face in hull_faces:
		var a = face[0]
		var b = face[1]
		var c = face[2]
		var d = face[3]

		# Calculate the plane equation
		var normal = (b - a).cross(c - a).normalized()
		var d_plane = normal.dot(a)

		# Check if the ray intersects the plane
		var denom = direction.dot(normal)
		if abs(denom) > 1e-6:
			var t = (d_plane - origin.dot(normal)) / denom
			if t >= 0:
				# Calculate the intersection point
				var intersection = origin + direction * t

				# Check if the intersection point is inside the quad
				var edge1 = b - a
				var edge2 = c - a
				var edge3 = d - a
				var vp = intersection - a
				var d00 = edge1.dot(edge1)
				var d01 = edge1.dot(edge2)
				var d11 = edge2.dot(edge2)
				var d20 = vp.dot(edge1)
				var d21 = vp.dot(edge2)
				var denom2 = d00 * d11 - d01 * d01
				var u = (d11 * d20 - d01 * d21) / denom2
				var v = (d00 * d21 - d01 * d20) / denom2

				if u >= 0 and v >= 0 and (u + v) <= 1:
					return true  # Ray intersects this face

	return false  # Ray does not intersect any face

# Function to filter out internal triangles
func filter_triangles(hull_faces: Array) -> Array:
	# Filter out duplicate faces
	var unique_faces = filter_duplicate_faces(hull_faces)

	# Filter out degenerate faces
	var valid_faces = filter_degenerate_faces(unique_faces)

	# Filter out internal faces
	var outer_faces = []

	for face in valid_faces:
		var a = face[0]
		var b = face[1]
		var c = face[2]
		var d = face[3]

		# Calculate the normal of the face
		var normal = (b - a).cross(c - a).normalized()

		# Calculate the centroid of the face
		var centroid = (a + b + c + d) / 4.0

		# Cast a ray from the centroid along the normal
		if not ray_intersects_hull(centroid + (normal * 0.01), normal, hull_faces):
			outer_faces.append(face)

	return outer_faces

# Function to create an ArrayMesh from filtered convex hull faces using quads
func create_mesh_from_hull(hull_faces: Array) -> ArrayMesh:
	var surface_tool = SurfaceTool.new()
	surface_tool.begin(Mesh.PRIMITIVE_TRIANGLES)

	# Add vertices, indices, and normals to the SurfaceTool
	for face in hull_faces:
		if face.size() == 4:  # Ensure it's a quad
			var a = face[0]
			var b = face[1]
			var c = face[2]
			var d = face[3]

			# Calculate the normal for this quad
			var normal = (b - a).cross(c - a).normalized()

			# Split the quad into two triangles and add vertices
			surface_tool.add_normal(normal)
			surface_tool.add_vertex(a)
			surface_tool.add_vertex(b)
			surface_tool.add_vertex(c)

			surface_tool.add_normal(normal)
			surface_tool.add_vertex(a)
			surface_tool.add_vertex(c)
			surface_tool.add_vertex(d)

	# Generate the mesh
	surface_tool.generate_normals()  # Ensure normals are calculated correctly
	surface_tool.index()  # Automatically generate indices
	var mesh = surface_tool.commit()
	return mesh

# Function to create a slightly opaque material
func create_opaque_material() -> Material:
	var material = SpatialMaterial.new()
	material.albedo_color = Color(1, 1, 1, 0.5)  # White color with 50% opacity
	material.flags_transparent = true  # Enable transparency
	return material

# Update function to generate the convex hull and create the mesh
func update():
	if not Engine.editor_hint:
		return  # Only run in the editor

	print("Starting convex hull generation...")

	# Remove the old mesh instance if it exists
	var old_mesh = get_node_or_null("__MESH")
	if old_mesh:
		old_mesh.name = "__REMOVED" # Do not remove, it breaks otherwise
		old_mesh.queue_free()

	# Get the points from the "Points" node
	var vec_points = []
	for point in get_node("Points").get_children():
		vec_points.append(point.global_transform.origin)
	var points = PoolVector3Array(vec_points)

	print("Computing convex hull...")
	var hull_faces = convex_hull_3d(points)
	print("Convex hull computed with ", hull_faces.size(), " faces.")

	print("Fixing face orientation...")
	hull_faces = fix_face_orientation(hull_faces)

	print("Filtering triangles...")
	var filtered_faces = filter_triangles(hull_faces)
	print("Filtered to ", filtered_faces.size(), " faces.")

	print("Creating mesh...")
	var mesh = create_mesh_from_hull(filtered_faces)

	# Create a MeshInstance and add it to the scene
	var mesh_instance = MeshInstance.new()
	mesh_instance.mesh = mesh
	mesh_instance.name = "__MESH"

	# Add a slightly opaque material to the mesh
	var material = create_opaque_material()
	mesh_instance.material_override = material

	add_child(mesh_instance)
	mesh_instance.owner = self

	print("Mesh generation complete.")

# Setter for the update property
func set_update(value):
	if value == false:
		return
	update()
