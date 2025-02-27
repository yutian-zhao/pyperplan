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
import builtins
from datetime import datetime
import pickle
from collections import Counter
import numpy as np

# from strips_hgn.planning import STRIPSProblem

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


def greedy_best_first_search(task, heuristic, use_relaxed_plan=False,
    max_search_time=float("inf"),
    heuristic_models = None,
    mode=None,
    **kwargs,
    ):
    """
    Searches for a plan in the given task using greedy best first search.

    @param task The task to be solved.
    @param heuristic A heuristic callable which computes the estimated steps
                     from a search node to reach the goal.
    """
    return astar_search(
        task, heuristic, ordered_node_greedy_best_first, use_relaxed_plan,
        max_search_time,
        heuristic_models,
        mode,
        **kwargs,
    )


def weighted_astar_search(
    task, heuristic, weight=5, use_relaxed_plan=False,
    max_search_time=float("inf"),
    heuristic_models = None,
    mode=None,
    **kwargs,
    ):
    """
    Searches for a plan in the given task using A* search.

    @param task The task to be solved.
    @param heuristic  A heuristic callable which computes the estimated steps.
                      from a search node to reach the goal.
    @param weight A weight to be applied to the heuristics value for each node.
    """
    return astar_search(
        task, heuristic, ordered_node_weighted_astar(weight), use_relaxed_plan,
        max_search_time,
        heuristic_models,
        mode,
        **kwargs,
    )

def retrain(checkpoint, problem, data_path):
    # model = STRIPSHGN.load_from_checkpoint(checkpoint)
    # model.train()
    # model._prediction_mode = False
    from train import train_wrapper
    from default_args import get_training_args, DomainAndProblemConfiguration
    from strips_hgn.models.strips_hgn import STRIPSHGN

    _CONFIGURATION = DomainAndProblemConfiguration(
        base_directory="",
        domain_pddl=problem.domain_pddl,
        problem_pddls=[problem.problem_pddl]
    )

    experiment_type = os.path.basename(os.path.dirname(checkpoint)).split('-strips')[0]
    _log.info(f"Retraining: {experiment_type}")
    
    model_path = train_wrapper(
        args=get_training_args(
            configurations=[_CONFIGURATION],
            max_training_time=2*60,
            num_folds=2,
            # batch_size=32,
            # learning_rate=0.005,
        ),
        experiment_type=experiment_type,
        mode=None,
        use_logging=False,
        strips_model_path = checkpoint,
        data_path = data_path
    )
    model = STRIPSHGN.load_from_checkpoint(model_path)
    model.eval()
    model.setup_prediction_mode()
    return model, model_path

def astar_search(
    task,
    heuristic,
    make_open_entry=ordered_node_astar,
    use_relaxed_plan=False,
    max_search_time=float("inf"),
    heuristic_models = None,
    mode=None,
    **kwargs,
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
    ## initialize
    # open list and closed list
    open = []
    state_cost = {task.initial_state: 0}
    node_tiebreaker = 0

    # initialize with root
    root = searchspace.make_root_node(task.initial_state)
    init_h = heuristic(root)
    node_h = {task.initial_state:init_h}
    heapq.heappush(open, make_open_entry(root, init_h, node_tiebreaker))
    _log.info("Initial h value: %f" % init_h)

    # get kwargs
    all = mode.get('all', False) if mode else False
    novel = mode.get('novel', 0) if mode else 0
    complement = mode.get('complement', 0) if mode else 0
    distance = mode.get('distance', 0) if mode else 0
    lifted = mode.get('lifted', 0) if mode else 0
    checkpoint = kwargs.get('checkpoint', None)
    problem = kwargs.get('problem', None)
    if checkpoint:
        from strips_hgn.training_data import StateValuePair
        retrain_interval = kwargs.get('retrain_interval', 2*60)
        experiment_dir = kwargs.get('experiment_dir', os.path.dirname(checkpoint))
        buffer = []
        data_amount = kwargs.get('data_amount', 600)
        resume_time = time.perf_counter()
        retrain_count = kwargs.get('retrain_count', 0)
        h_counter = Counter()

    _log.info("Mode: {}".format(mode))

    if heuristic_models:
        compare_list = []
    if all:
        all_pairs = []
    elif novel:
        # file = builtins.open(os.path.join(os.getcwd(), "novelty.log"), 'a')
        # file.write("total number of facts of {}: {}. (n*(n-1)={}).\n".format(
        #     task.name, len(task.facts), len(task.facts)*(len(task.facts)-1)))
        novel_pairs = []
        num_novelty_1 = 0
        num_novelty_2 = 0
        num_novelty_inf = 0
        single_tuples = set()
        double_tuples = set() # initial state is added to the queue first.
        complement_pairs = []
            
    besth = float("inf")
    counter = 0 # nodes dequeued
    expansions = 0 # nodes expanded

    # Number of heuristic calls, include the initial call for root note
    heuristic_calls = 1

    # Used so we can interrupt the search
    start_time = time.perf_counter()

    while open:
        ## Check whether max search time exceeded
        elapsed_time = time.perf_counter() - start_time
        if elapsed_time >= max_search_time:
            metrics = SearchMetrics(
                nodes_expanded=expansions,
                plan_length=-1,
                heuristic_calls=heuristic_calls,
                heuristic_val_for_initial_state=init_h,
                search_time=elapsed_time,
                search_state=SearchState.timed_out,
                checkpoint = checkpoint,
            )
            _log.warning("Search timed out")
            _log.info("%d Nodes expanded" % expansions)
            _log.info("%d times heuristic called" % heuristic_calls)

            if heuristic_models:
                return compare_list, metrics
            elif all:
                return all_pairs, metrics
            elif novel:
                _log.info("Number of states: {}; novelty 1: {}; novelty 2: {}; novelty inf: {}.".format( 
                    (num_novelty_1+num_novelty_2+num_novelty_inf),num_novelty_1, num_novelty_2, num_novelty_inf))
                if complement:
                    _log.info(f"Number of complement pairs: {len(complement_pairs)}.")
                return novel_pairs+complement_pairs, metrics
            else:
                return [], metrics
        
        ## pop buffer and invoke retraining after the time of 'retrain interval' passed
        if checkpoint and time.perf_counter() - resume_time >= retrain_interval and len(buffer)>=data_amount:
            problem_to_state_value_pairs = {problem: []}
            _log.info(f"Uniform sampling {data_amount} pairs among {len(buffer)} pairs in the buffer.")
            probs = np.array([1/h_counter[pair.value] for pair in buffer])
            probs = probs/np.sum(probs)
            state_value_pairs = np.random.choice(buffer, size=data_amount, replace=False, p=probs).tolist()
            for state_value_pair in state_value_pairs:
                problem_to_state_value_pairs[problem].append(state_value_pair)
            # _log.info(f"Saving first {data_amount} pairs among {len(buffer)} pairs in the buffer.")
            # for i in range(data_amount):
            #     _, state_value_pair = heapq.heappop(buffer)
            #     problem_to_state_value_pairs[problem].append(StateValuePair(state=state_value_pair[0], target=state_value_pair[1], value=state_value_pair[2]))
            os.path.dirname(checkpoint)
            data_path = os.path.join(experiment_dir, 'training_data-'+datetime.now().strftime("%m-%d-%H-%M-%S")+'.pkl')
            pickle.dump(problem_to_state_value_pairs, builtins.open(data_path, "wb" ))
            _log.info(f"Saved data at {data_path}.")
            _log.info(f"Start retraining {retrain_count}.")
            model, model_path = retrain(checkpoint, problem, data_path)
            _log.info(f"Finish retraining, checkpoint is located at {model_path}.")
            # replace model
            checkpoint = model_path
            kwargs['checkpoint'] = checkpoint
            heuristic.model = model
            resume_time = time.perf_counter()
            retrain_count += 1
            kwargs['retrain_count'] = retrain_count
            os.remove(data_path)
            # restart
            _log.info("Restart search with retrained model.")
            _log.info("%d Nodes expanded" % expansions)
            _log.info("%d times heuristic called" % heuristic_calls)
            return greedy_best_first_search(
                task=task,
                heuristic=heuristic,
                # make_open_entry=make_open_entry,
                use_relaxed_plan=use_relaxed_plan,
                max_search_time=max_search_time-(time.perf_counter() - start_time),
                heuristic_models = heuristic_models,
                mode=mode,
                **kwargs,
            )

        ## pop node
        (f, h, _tie, pop_node) = heapq.heappop(open)
        if h < besth:
            besth = h
            # debug -> info
            _log.info(f"Found new best h: {besth} after {counter} expansions")

        pop_state = pop_node.state

        ## dequeue node
        # Only expand the node if its associated cost (g value) is the lowest
        # cost known for this state. Otherwise we already found a cheaper
        # path after creating this node and hence can disregard it.
        if state_cost[pop_state] == pop_node.g:
            expansions += 1

            # If asked to find all paths, collect paths when dequeuing.
            if all:
                all_pairs+=pop_node.extract_state_value_pairs(distance=distance)
            if novel:
                # compute novelty when dequeuing instead of expanding
                single_tuples, double_tuples, novelty, novel_set = searchspace.compute_novelty(single_tuples, double_tuples, pop_state)
                pop_node.novelty = novelty
                pop_node.novel_set = novel_set
                if novelty==1 and novelty<=novel:
                    num_novelty_1+=1
                    novel_pairs+=pop_node.extract_state_value_pairs(distance=distance, novel=novel, lifted=lifted)
                elif novelty==2 and novelty<=novel:
                    num_novelty_2+=1
                    novel_pairs+=pop_node.extract_state_value_pairs(distance=distance, novel=novel, lifted=lifted)
                else:
                    num_novelty_inf+=1
                    if complement > 0:
                        if len(complement_pairs)<len(novel_pairs)*complement/100:
                            complement_pairs+=pop_node.extract_state_value_pairs(distance=distance, novel=False, lifted=False)
            
            # data selection
            if checkpoint:
                for pair in pop_node.extract_state_value_pairs(distance=distance, novel=novel, lifted=lifted):
                    # criterion 1: -|target.g - |target.h-source.h|| 
                    # heapq.heappush(buffer, (-abs(pair[-1]- abs(node_h[pair[0]]-node_h[pair[1]])), pair))
                    # criterion 2: -|target.g - |h(source, target)||
                    # heapq.heappush(buffer, (-abs(pair[-1]- abs(heuristic(pair[0], pair[1]))), pair))
                    # criterion 3: -|target.g - |target.h-source.h||/target.g 
                    # heapq.heappush(buffer, (-abs(pair[-1]- abs(node_h[pair[0]]-node_h[pair[1]]))/(pair[-1]+1), pair))
                    # criterion 4: uniform sampling based on h
                    buffer.append(StateValuePair(state=pair[0], target=pair[1], value=pair[2]))
                    h_counter[pair[-1]] += 1

            ## check node
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
                    checkpoint = checkpoint,
                )

                if heuristic_models:
                    return compare_list, metrics
                elif all:
                    _log.info("Returning all pairs.")
                    return all_pairs, metrics
                elif novel:
                    # file.write("total number of states: {}; novelty 1: {}; novelty 2: {}; nonnovel: {}.\n".format(
                    #     num_novelty_1+num_novelty_2+num_novelty_inf, num_novelty_1, num_novelty_2, num_novelty_inf))
                    _log.info("Remaining {} states in the queue. Collecting novel pairs, lifted is {}.".format(len(open), lifted))
                    remain_novel_nodes = 0
                    while open:
                        (_, _, _, node) = heapq.heappop(open)
                        single_tuples, double_tuples, novelty, novel_set = searchspace.compute_novelty(single_tuples, double_tuples, node.state)
                        node.novelty = novelty
                        node.novel_set = novel_set
                        if novelty==1 and novelty<=novel:
                            num_novelty_1+=1
                            remain_novel_nodes+=1
                            novel_pairs+=node.extract_state_value_pairs(distance=distance, novel=novel, lifted=lifted)
                        elif novelty==2 and novelty<=novel:
                            num_novelty_2+=1
                            remain_novel_nodes+=1
                            novel_pairs+=node.extract_state_value_pairs(distance=distance, novel=novel, lifted=lifted)
                        else:
                            num_novelty_inf+=1
                            if complement > 0:
                                old_succ_g = state_cost.get(node.state, float("inf"))
                                if node.g < old_succ_g and len(complement_pairs)<len(novel_pairs)*complement/100:
                                    complement_pairs+=pop_node.extract_state_value_pairs(distance=distance, novel=False, lifted=False)
                    _log.info("Remaining {} novel states in the queue.".format(remain_novel_nodes))
                    _log.info("Total number of states (threhold {}): {}; novelty 1: {}; novelty 2: {}; nonnovel: {}.\n".format(
                        novel, num_novelty_1+num_novelty_2+num_novelty_inf, num_novelty_1, num_novelty_2, num_novelty_inf))
                    if complement:
                        _log.info(f"Number of complement pairs: {len(complement_pairs)}.")
                    return novel_pairs+complement_pairs, metrics
                else:
                    return [(p[0], None, p[2]) for p in pop_node.extract_state_value_pairs(distance=distance)], metrics                

            rplan = None
            if use_relaxed_plan:
                (rh, rplan) = heuristic.calc_h_with_plan(
                    searchspace.make_root_node(pop_state)
                )
                _log.debug("relaxed plan %s " % rplan)

            if heuristic_models:
                states_and_hs = [[] for i in range(len(heuristic_models)+2)]
                states_and_hs.append(pop_node.action.name if pop_node.action else '') # The action which produced the state.

            ## expand and push node 
            # siblings_h = []
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
                    node_h[succ_state] = h
                    # siblings_h.append(h)
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
                        # NOTE: h+g; g if use GBFS
                        states_and_hs[1].append(h+succ_node.g)
                        for idx, heuristic_model in enumerate(heuristic_models):
                            states_and_hs[idx+2].append(heuristic_model(succ_node)+succ_node.g)
            # _log.info(f"node h {node_h[pop_state]} siblings h {siblings_h}.")

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
        checkpoint = checkpoint,
    )

    if heuristic_models:
        return compare_list, metrics
    elif all:
        return all_pairs, metrics
    elif novel:
        _log.info("Number of states: {}; novelty 1: {}; novelty 2: {}; novelty inf: {}.".format( 
                    (num_novelty_1+num_novelty_2+num_novelty_inf),num_novelty_1, num_novelty_2, num_novelty_inf))
        if complement:
            _log.info(f"Number of complement pairs: {len(complement_pairs)}.")
        return novel_pairs+complement_pairs, metrics
    else:
        return None, metrics
