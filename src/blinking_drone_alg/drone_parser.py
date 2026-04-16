import csv

"""
[
    [(x, y, color, flag), ...], # drone 1
    [(x, y, color, flag), ...], # drone 2
]
"""

class DroneParser:
    @staticmethod
    def load_csv(file_path: str):
        with open(file_path, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=' ', quotechar='|')

            for row in reader:
                print(', '.join(row))
        pass
