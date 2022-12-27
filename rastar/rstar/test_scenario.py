import random

from dataclasses import dataclass
from typing import List
from .utils import Map


@dataclass
class Test:
    start_i: int
    start_j: int
    goal_i: int
    goal_j: int
    result: float

    @staticmethod
    def from_scenario_line(line):
        splitted = line.split('\t')
        return Test(int(splitted[5]), int(splitted[4]), int(splitted[7]), int(splitted[6]), float(splitted[8]))


@dataclass
class TestScenario:
    map: Map
    tests: List[Test]
    name: str

    @staticmethod
    def from_files(map_file: str, name: str, test_num=100):
        result = TestScenario(Map(), [], name)
        with open(map_file, 'r') as f:
            f.readline()
            height = int(f.readline().split(' ')[1])
            width = int(f.readline().split(' ')[1])
            f.readline()
            cell_str = f.read()
            result.map.read_from_string(cell_str, width, height)
        with open(map_file+'.scen') as f:
            lines = f.read().split('\n')[1:-1]
            if test_num > len(lines):
                raise Exception('Not enough tests')
            lines_to_use = [lines[i * len(lines) // test_num] for i in range(test_num)]
            for line in lines_to_use:
                result.tests.append(Test.from_scenario_line(line))
        random.shuffle(result.tests)
        return result