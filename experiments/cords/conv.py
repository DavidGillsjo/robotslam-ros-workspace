from affine import Affine

# Local cordinates
gps_points_local = [[0.0, 0.0], [1.0, 1.0]]

# GPS reference points
gps_points_geo = [[55.708955, 13.210666], [55.709067, 13.210856]]

# Create the transformation matrix
transform = Affine.from_tiepoints(gps_points_local,gps_points_geo)

print('Transformation Matrix:')
print(transform.trans_matrix)

# Convert the following points
local_point_cloud = [[0.5, 0.5]]
transform.transform(local_point_cloud)
