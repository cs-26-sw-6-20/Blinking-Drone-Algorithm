import pytest
from pathlib import Path


from blinking_drone_alg.math_utils import distance



from math import sqrt





@pytest.fixture(params=[
    # Format: (start, target, expected_distance)
    ((0, 0, 0), (0, 0, 0), 0.0),
    ((1, 2, 3), (4, 5, 6), sqrt(27)),
    ((-1, -2, -3), (-4, -5, -6), sqrt(27)),
    ((1.5, 2.5, 3.5), (4.5, 5.5, 6.5), sqrt(27))
])

def distance_test_data(request):
    return request.param


class TestDroneshow:


    #distance tests
    def test_distance_calculation(self, distance_test_data):
        point_a, point_b, expected = distance_test_data
        
        result = distance(point_a, point_b)
        assert pytest.approx(result) == expected

    def test_invalid_tuple_length(self):
        with pytest.raises(IndexError):
            distance((1, 2), (3, 4, 5)),

    def none_distance_test(self):
        with pytest.raises(TypeError):
            distance(None, (4, 5, 6)),
            distance((1, 2, 3), None)
            distance((None, 2, 3), (4, 5, 6)),
            distance(("a", 2, 3), (4, 5, 6)),
            distance(([1, 2, 3]), (4, 5, 6))
        
    

    
