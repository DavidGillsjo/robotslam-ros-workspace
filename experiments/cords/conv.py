import json
from pprint import pprint
from affine import Affine

with open('config.json') as data:
    config = json.load(data)

print('Config:')
pprint(config)

# Create the transformation matrix
transform = Affine.from_tiepoints(config['local_points'], config['reference_points'])

print('Transformation Matrix:')
print(transform.trans_matrix)

# Convert the following points
local_point_cloud = [[0.5, 0.5]]
transform.transform(local_point_cloud)
