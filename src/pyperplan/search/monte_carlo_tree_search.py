"""
Implements the novelty search algorithm.
"""

from collections import deque
import logging
import heapq
import numpy as np 
import time # add time limit
import os # for debugging

from pyperplan.search import searchspace
from pyperplan.task import Task
from pyperplan.search.metrics import SearchMetrics, SearchState


_log = logging.getLogger(__name__)

def selection(root, selection_function, parameter):
    current = root
    while len(current.children):
        # _log.info("current is {}".format(current.tiebreaker))
        current = max(current.children, key=lambda x: (ucb1(x, parameter), -x.h, -x.tiebreaker))
        # _log.info("the following's ucb1 is {}".format(ucb1(current, parameter)))
        # current = current.children[np.argmax([selection_function(child, parameter) for child in current.children])]
    return current

def simulation(task, node):
    # only support step==0
    # further add gbfs and random within 'step'
    if task.goal_reached(node.state):
        return 0
    return node.h

# parameter and v value.
def ucb1(mcts_node, parameter):
    if mcts_node.parent:
        # initialize v value -> num_visits
        # mcts_node.parent.num_visits and num_visits
        if mcts_node.num_visits:
            return parameter*np.sqrt(np.log(mcts_node.parent.num_visits)/mcts_node.num_visits)-mcts_node.v
        else:
            return float('inf')
    else:
        return None

# Multiprocess programming. Shared buffer.
def monte_carlo_tree_search(
    task, 
    heuristic,
    max_search_time=float("inf"), 
    mode=None,
    selection_function=ucb1,
    parameter=np.sqrt(2)
):
    start_time = time.perf_counter()
    nodes_count = 1
    expansions = 0
    root = searchspace.make_mcts_root_node(task.initial_state)
    root.h = heuristic(root)
    _log.info("Initial h value: %f" % root.h)
    root.tiebreaker = nodes_count
    # may need to change if lazy
    goal_node = False
    # currently eager; make change to state_cost = {task.initial_state: 0}
    closed = {task.initial_state}
    # perform iteration of rounds
    metrics = None
    while not goal_node:
        # check if time out
        elapsed_time = time.perf_counter() - start_time
        if elapsed_time >= max_search_time:
            metrics = SearchMetrics(
                nodes_expanded=expansions,
                plan_length=-1,
                heuristic_calls=nodes_count,
                heuristic_val_for_initial_state=root.h,
                search_time=elapsed_time,
                search_state=SearchState.timed_out,
            )
            _log.warning("Search timed out")
            _log.info("%d Nodes expanded" % expansions)
            _log.info("%d times heuristic called" % nodes_count)
            return [], metrics
        
        # selection
        leaf = selection(root, selection_function, parameter)
        # _log.info("Select leaf {}".format(leaf.tiebreaker))

        # expansion
        if leaf.num_visits:
            expansions += 1
            # _log.info("Expanding leaf {}".format(leaf.tiebreaker))
            new_children = 0
            for op, succ_state in task.get_successor_states(leaf.state):
                # initialize child node
                if succ_state not in closed:
                    new_children += 1
                    closed.add(succ_state)
                    nodes_count += 1
                    child = searchspace.make_mcts_child_node(leaf, op, succ_state)
                    child.h = heuristic(child)
                    child.tiebreaker = nodes_count
                    leaf.children.append(child)
                    # _log.info("Adding leaf {}".format(leaf.tiebreaker))
                # check goal state
                if task.goal_reached(succ_state):
                    goal_node = child
                    _log.info("Goal reached. Start extraction of solution.")
                    _log.info("%d Nodes expanded" % expansions)
                    # currently nodes_count=heuristic calls
                    _log.info("%d times heuristic called" % nodes_count)
                    sol = goal_node.extract_solution()
                    _log.info("Solution length: {} actions.".format(len(sol)))
                    # Create metrics
                    metrics = SearchMetrics(
                        nodes_expanded=expansions,
                        plan_length=len(sol),
                        heuristic_calls=nodes_count,
                        heuristic_val_for_initial_state=root.h,
                        search_time=time.perf_counter() - start_time,
                        search_state=SearchState.success,
                    )
            if not new_children:
                _log.info("Found Deadend: {}".format(leaf.tiebreaker))
                _log.info("Siblings are: {}".format([n.tiebreaker for n in leaf.parent.children]))
                leaf.v = float('inf')
                # leaf.h = float('inf')
            else:
                leaf = selection(leaf, selection_function, parameter)
                # simulation
                leaf.v = simulation(task, leaf)
                # _log.info("Simulate leaf {}".format(leaf.tiebreaker))
        else:
            leaf.v = simulation(task, leaf)

        # backpropagation
        leaf.num_visits += 1
        if leaf.parent:
            backup_node = leaf.parent
            if min([child.v for child in leaf.parent.children if child.v is not None])<leaf.v:
                while backup_node:
                    backup_node.num_visits += 1
                    backup_node = backup_node.parent
            else:
                # assume uniform cost
                # v default to be inf
                while backup_node:
                    backup_node.num_visits += 1
                    # currently use min; mean is also triable.
                    backup_node.v = 1+min([child.v for child in backup_node.children if child.v is not None])
                    backup_node = backup_node.parent

    # what to do when task is unsolvable
    if goal_node:
        return [(p[0], None, p[2]) for p in goal_node.extract_state_value_pairs()], metrics
    else:
        _log.info("No operators left. Task unsolvable.")
        _log.info("%d Nodes expanded" % expansions)
        return [], metrics

    