#
# This file is part of pyperplan.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>
#

"""
Implements the A* (a-star) and weighted A* search algorithm.
"""

import heapq
import logging
import time

from pyperplan.search import searchspace
from pyperplan.task import Task
from pyperplan.search.metrics import SearchMetrics, SearchState

import os

_log = logging.getLogger(__name__)

def ordered_node_astar(node, h, node_tiebreaker):
    """
    Creates an ordered search node (basically, a tuple containing the node
    itself and an ordering) for A* search.

    @param node The node itself.
    @param heuristic A heuristic function to be applied.
    @param node_tiebreaker An increasing value to prefer the value first
                           inserted if the ordering is the same.
    @returns A tuple to be inserted into priority queues.
    """
    f = node.g + h
    return (f, h, node_tiebreaker, node)


def ordered_node_weighted_astar(weight):
    """
    Creates an ordered search node (basically, a tuple containing the node
    itself and an ordering) for weighted A* search (order: g+weight*h).

    @param weight The weight to be used for h
    @param node The node itself
    @param h The heuristic value
    @param node_tiebreaker An increasing value to prefer the value first
                           inserted if the ordering is the same
    @returns A tuple to be inserted into priority queues
    """
    """
    Calling ordered_node_weighted_astar(42) actually returns a function (a
    lambda expression) which is the *actual* generator for ordered nodes.
    Thus, a call like
        ordered_node_weighted_astar(42)(node, heuristic, tiebreaker)
    creates an ordered node with weighted A* ordering and a weight of 42.
    """
    return lambda node, h, node_tiebreaker: (
        node.g + weight * h,
        h,
        node_tiebreaker,
        node,
    )


def ordered_node_greedy_best_first(node, h, node_tiebreaker):
    """
    Creates an ordered search node (basically, a tuple containing the node
    itself and an ordering) for greedy best first search (the value with lowest
    heuristic value is used).

    @param node The node itself.
    @param h The heuristic value.
    @param node_tiebreaker An increasing value to prefer the value first
                           inserted if the ordering is the same.
    @returns A tuple to be inserted into priority queues.
    """
    f = h
    return (f, h, node_tiebreaker, node)


def greedy_best_first_search(task, heuristic, use_relaxed_plan=False):
    """
    Searches for a plan in the given task using greedy best first search.

    @param task The task to be solved.
    @param heuristic A heuristic callable which computes the estimated steps
                     from a search node to reach the goal.
    """
    return astar_search(
        task, heuristic, ordered_node_greedy_best_first, use_relaxed_plan
    )


def weighted_astar_search(task, heuristic, weight=5, use_relaxed_plan=False):
    """
    Searches for a plan in the given task using A* search.

    @param task The task to be solved.
    @param heuristic  A heuristic callable which computes the estimated steps.
                      from a search node to reach the goal.
    @param weight A weight to be applied to the heuristics value for each node.
    """
    return astar_search(
        task, heuristic, ordered_node_weighted_astar(weight), use_relaxed_plan
    )


def astar_search(
    task,
    heuristic,
    make_open_entry=ordered_node_astar,
    use_relaxed_plan=False,
    max_search_time=float("inf"),
    heuristic_models = None,
    remove_trivial = False,
    mode=['solution'],
):
    """
    Searches for a plan in the given task using A* search. This function will return a list of state value pairs.

    @param task The task to be solved
    @param heuristic  A heuristic callable which computes the estimated steps
                      from a search node to reach the goal.
    @param make_open_entry An optional parameter to change the bahavior of the
                           astar search. The callable should return a search
                           node, possible values are ordered_node_astar,
                           ordered_node_weighted_astar and
                           ordered_node_greedy_best_first with obvious
                           meanings.
    @param max_search_time Maximum search time in seconds
    @param heuristic_models Heuristics to be evaluated.
    @param mode State value collecting mode.
                'solution' collects ((state, goal), value) pairs along the solution path.
                'all' collects all ((state, state), value) pairs. 
                'novel' collects ((start, novel state), value) pairs.
                'nontrivial' whether remove pairs with value less than 2.
    """
    open = []
    state_cost = {task.initial_state: 0}
    node_tiebreaker = 0

    root = searchspace.make_root_node(task.initial_state)
    init_h = heuristic(root)
    heapq.heappush(open, make_open_entry(root, init_h, node_tiebreaker))
    _log.info("Initial h value: %f" % init_h)

    if heuristic_models:
        compare_list = []
    if 'all' in mode:
        all_pairs = []
    if 'novel' in mode:
        novel_pairs = []
        num_novelty_1 = 0
        num_novelty_2 = 0
        num_novelty_inf = 0
        single_tuples = set()
        double_tuples = set() # initial state is added to the queue first.
    remove_trivial = 'nontrivial' in mode

    besth = float("inf")
    counter = 0
    expansions = 0

    # Number of heuristic calls, include the initial call for root note
    heuristic_calls = 1

    # Used so we can interrupt the search
    start_time = time.perf_counter()

    while open:
        # Check whether max search time exceeded
        elapsed_time = time.perf_counter() - start_time
        if elapsed_time >= max_search_time:
            metrics = SearchMetrics(
                nodes_expanded=expansions,
                plan_length=-1,
                heuristic_calls=heuristic_calls,
                heuristic_val_for_initial_state=init_h,
                search_time=elapsed_time,
                search_state=SearchState.timed_out,
            )
            _log.warning("Search timed out")
            _log.info("%d Nodes expanded" % expansions)
            _log.info("%d times heuristic called" % heuristic_calls)

            if heuristic_models:
                return compare_list, metrics
            elif 'all' in mode:
                return all_pairs, metrics
            elif 'novel' in mode:
                print("total number: ", num_novelty_1+num_novelty_2+num_novelty_inf, 
                    'novelty 1: ', num_novelty_1, 'novelty 2: ', num_novelty_2, 'novelty inf: ', num_novelty_inf)
                return novel_pairs, metrics
            else:
                return [], metrics

        (f, h, _tie, pop_node) = heapq.heappop(open)
        if h < besth:
            besth = h
            _log.debug("Found new best h: %d after %d expansions" % (besth, counter))

        pop_state = pop_node.state

        # Only expand the node if its associated cost (g value) is the lowest
        # cost known for this state. Otherwise we already found a cheaper
        # path after creating this node and hence can disregard it.
        if state_cost[pop_state] == pop_node.g:
            expansions += 1

            # If asked to find all paths, collect paths when dequeuing.
            if 'all' in mode:
                all_pairs+=pop_node.extract_state_value_pairs(remove_trivial=remove_trivial)
            if 'novel' in mode:
                single_tuples, double_tuples, novelty = searchspace.compute_novelty(single_tuples, double_tuples, pop_state)
                if novelty==1:
                    num_novelty_1+=1
                    novel_pairs+=pop_node.extract_state_value_pairs(remove_trivial=remove_trivial)
                elif novelty==2:
                    num_novelty_2+=1
                    novel_pairs+=pop_node.extract_state_value_pairs(remove_trivial=remove_trivial)
                else:
                    num_novelty_inf+=1

            if task.goal_reached(pop_state):
                _log.info("Goal reached. Start extraction of solution.")
                _log.info("%d Nodes expanded" % expansions)
                _log.info("%d times heuristic called" % heuristic_calls)
                # _log.info("solution: {}".format(pop_node.extract_solution()))
                sol = pop_node.extract_solution()
                _log.info("Solution length: {} actions.".format(len(sol)))

                # Create metrics
                metrics = SearchMetrics(
                    nodes_expanded=expansions,
                    plan_length=len(sol),
                    heuristic_calls=heuristic_calls,
                    heuristic_val_for_initial_state=init_h,
                    search_time=time.perf_counter() - start_time,
                    search_state=SearchState.success,
                )

                if heuristic_models:
                    print(sol)
                    return compare_list, metrics
                elif 'all' in mode:
                    return all_pairs, metrics
                elif 'novel' in mode:
                    print("total number: ", num_novelty_1+num_novelty_2+num_novelty_inf, 
                        'novelty 1: ', num_novelty_1, 'novelty 2: ', num_novelty_2, 'novelty inf: ', num_novelty_inf)
                    return novel_pairs, metrics
                else:
                    return pop_node.extract_state_value_pairs(remove_trivial=remove_trivial), metrics                

            rplan = None
            if use_relaxed_plan:
                (rh, rplan) = heuristic.calc_h_with_plan(
                    searchspace.make_root_node(pop_state)
                )
                _log.debug("relaxed plan %s " % rplan)

            if heuristic_models:
                states_and_hs = [[] for i in range(len(heuristic_models)+2)]
                states_and_hs.append(pop_node.action.name if pop_node.action else '') # The action which produced the state.

            for op, succ_state in task.get_successor_states(pop_state):
                if use_relaxed_plan:
                    if rplan and not op.name in rplan:
                        # ignore this operator if we use the relaxed plan
                        # criterion
                        _log.debug(
                            "removing operator %s << not a "
                            "preferred operator" % op.name
                        )
                        continue
                    else:
                        _log.debug("keeping operator %s" % op.name)

                succ_node = searchspace.make_child_node(pop_node, op, succ_state)

                old_succ_g = state_cost.get(succ_state, float("inf"))
                if succ_node.g < old_succ_g:
                    h = heuristic(succ_node)
                    heuristic_calls += 1

                    if h == float("inf"):
                        # don't bother with states that can't reach the goal anyway
                        continue

                    # We either never saw succ_state before, or we found a
                    # cheaper path to succ_state than previously.
                    node_tiebreaker += 1
                    heapq.heappush(open, make_open_entry(succ_node, h, node_tiebreaker))
                    state_cost[succ_state] = succ_node.g

                    if heuristic_models:
                        # heuristics=["gripper_ori_90", "gripper_all_90", "gripper_all_180"]
                        # checkpoints = [os.path.join("../results/", path, "model-best.ckpt") for path in heuristics] 
                        # heuristic_models = [model_to_heuristics(checkpoint, problem)for checkpoint in checkpoints]
                        states_and_hs[0].append(op.name)
                        states_and_hs[1].append(h+succ_node.g)
                        for idx, heuristic_model in enumerate(heuristic_models):
                            states_and_hs[idx+2].append(heuristic_model(succ_node)+succ_node.g)

            if heuristic_models:
                # if len(states_and_hs[0])!=0:
                compare_list.append(states_and_hs)

        counter += 1
    _log.info("No operators left. Task unsolvable.")
    _log.info("%d Nodes expanded" % expansions)

    # Create metrics
    metrics = SearchMetrics(
        nodes_expanded=expansions,
        plan_length=-1,
        heuristic_calls=heuristic_calls,
        heuristic_val_for_initial_state=init_h,
        search_time=time.perf_counter() - start_time,
        search_state=SearchState.failed,
    )
    return None, metrics
