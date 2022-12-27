import math

from datetime import datetime
from .rstar import rstar, rstar_metrics
from .test_scenario import TestScenario
from .test_results import AlgoTestResults, TestResult
from tqdm import tqdm


def run_test(results_data: AlgoTestResults, test_scenario: TestScenario):
    for test in tqdm(test_scenario.tests):
        start_time = datetime.now()
        found, node, metrics = results_data.algo(test_scenario.map, test.start_i, test.start_j, test.goal_i, test.goal_j, **results_data.kwargs)
        if not found:
            results_data.errors += 1
            continue
        length = node.g
        if not math.isclose(1000+length, 1000+test.result):
            results_data.errors += 1
        results_data.test_results.append(TestResult(length, test.result, metrics.total_expanded, metrics.total_steps, metrics.total_nodes_created, metrics.total_max_len, datetime.now() - start_time, test_scenario.name))


class TestRunner:
    def __init__(self, algo_dicts, scenarios):
        self.datas = []
        for kwargs in algo_dicts:
            label = kwargs['label']
            del kwargs['label']
            data = AlgoTestResults(label, rstar, kwargs)
            self.datas.append(data)
        self.scenarios = []
        for scenario_tuple in scenarios:
            scenario = TestScenario('', '', 0).from_files(*scenario_tuple)
            self.scenarios.append(scenario)

    def run(self):
        for result in self.datas:
            result.prune()
            for scenario in self.scenarios:
                run_test(result, scenario)
            result.make_df()