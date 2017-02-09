import numpy as N

# http://gis.stackexchange.com/questions/63107/using-proj-4-library-to-transform-from-local-coordinate-system-coordinates-to-gl/95147
def augment(a):
    """Add a final column of ones to input data"""
    arr = N.ones((a.shape[0], a.shape[1] + 1))
    arr[:, :-1] = a
    return arr

class Affine(object):
    def __init__(self, array=None):
        self.trans_matrix = array

    def transform(self, points):
        """Transform locally projected data using transformation matrix"""
        return N.dot(augment(N.array(points)), self.trans_matrix)

    @classmethod
    def from_tiepoints(cls, fromCoords, toCoords):
        "Produce affine transform by ingesting local and georeferenced coordinates for tie points"""
        fromCoords = augment(N.array(fromCoords))
        toCoords = N.array(toCoords)
        trans_matrix, residuals, rank, sv = N.linalg.lstsq(fromCoords, toCoords)

        affine = cls(trans_matrix)  # Setup affine transform from transformation matrix
        sol = N.dot(fromCoords, affine.trans_matrix)  # Compute model solution
        print "Pixel errors:"
        print (toCoords - sol)
        return affine
