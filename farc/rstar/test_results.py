import numpy as np
import pandas as pd

from typing import List, Optional, Callable, Dict
from datetime import datetime, timedelta
from dataclasses import dataclass, asdict, field


@dataclass
class TestResult:
    length: float
    expected_length: float
    expansions: int
    steps: int
    nodes_created: int
    memory: int
    time: timedelta
    map_name: str


@dataclass
class AlgoTestResults:
    label: str
    algo: Callable
    kwargs: Dict
    errors: int = 0
    test_results: List[TestResult] = field(default_factory=list)
    test_results_df: Optional[pd.DataFrame] = None

    def make_df(self):
        self.test_results_df = pd.DataFrame([asdict(res) for res in self.test_results], columns=list(TestResult.__annotations__.keys()))
        self.test_results_df['time'] = np.array([time.total_seconds() for time in self.test_results_df['time']])
        self.test_results_df = self.test_results_df.sort_values(by='length')
        
    def prune(self):
        self.errors: int = 0
        self.test_results: List[TestResult] = []
        self.test_results_df: Optional[pd.DataFrame] = None
