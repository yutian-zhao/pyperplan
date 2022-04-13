from enum import Enum
from typing import NamedTuple


class SearchState(Enum):
    # Plan found
    success = "success"
    # No plan found
    failed = "failed"
    # Search Timed out
    timed_out = "timed_out"


class SearchMetrics(NamedTuple):

    nodes_expanded: int
    plan_length: int
    heuristic_calls: int
    heuristic_val_for_initial_state: float
    search_time: float
    search_state: SearchState
    # overkill
    checkpoint: str