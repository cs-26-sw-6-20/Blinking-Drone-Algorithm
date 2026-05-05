import pytest
from blinking_drone_alg.droneshow import DroneshowModifier


class TestHungarianAlg:
    def test_hungarian_algorithm(self):
        
        distance_matrix = [
            [5, 1, 0],
            [5, 5, 0],
            [1, 1.1, 0]
        ]
          
        expectedOutput = [(0,1),(1,2),(2,0)]
        
        result = DroneshowModifier.HungarianALG(distance_matrix)

        assert len(result) == len(expectedOutput)
        assert result == expectedOutput
        
        result_normalized = [(int(r), int(c)) for r, c in result]
        
        # Compare the actual tuple values
        assert result_normalized == expectedOutput, \
            f"Expected {expectedOutput}, but got {result_normalized}"
        
        distance_matrix = [
            [5, 1, 0],
            [5, 5, 0],
            [1, 1.1, 0]
        ]
                
