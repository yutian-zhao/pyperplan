"""
Implements the novelty search algorithm.
"""

from collections import deque
import logging
import heapq

from pyperplan.search import searchspace

# add time limit
import time
import os # for debugging


def novelty_search(planning_task, max_search_time=float("inf"), mode=None):
    """
    Searches for a plan on the given task using breadth first search and
    duplicate detection.

    @param planning_task: The planning task to solve.
    @return: The solution as a list of operators or None if the task is
    unsolvable.
    """
    priority_queue = []
    node_tiebreaker = 0

    # counts the number of loops (only for printing)
    iteration = 0
    # fifo-queue storing the nodes which are next to explore
    root = searchspace.make_root_node(planning_task.initial_state)
    heapq.heappush(priority_queue, (1, node_tiebreaker, root))
    # set storing the explored nodes, used for duplicate detection
    closed = {planning_task.initial_state}

    all = mode.get('all', False) if mode else False
    novel = mode.get('novel', 2) if mode else 2
    distance = mode.get('distance', 0) if mode else 0
    lifted = mode.get('lifted', False) if mode else False

    # if all:
    #     all_pairs = []
    # if novel:
        # file = open(os.path.join(os.getcwd(), "novelty.log"), 'a')
        # file.write("total number of facts of {}: {}. (n*(n-1)={}).\n".format(planning_task.name, len(planning_task.facts), len(planning_task.facts)*(len(planning_task.facts)-1)))
    novel_pairs = []
    num_novelty_1 = 0
    num_novelty_2 = 0
    num_novelty_inf = 0
    single_tuples = set()
    double_tuples = set()

    start_time = time.perf_counter()

    while priority_queue:
        elapsed_time = time.perf_counter() - start_time
        if elapsed_time >= max_search_time:
            logging.info("Search timed out")
            logging.info("search_time: %d" % elapsed_time)
            if all:
            #     return all_pairs
            # if novel:
                # file.write("total number of states: {}; novelty 1: {}; novelty 2: {}; nonnovel: {}.\n".format(
                #     num_novelty_1+num_novelty_2+num_novelty_inf, num_novelty_1, num_novelty_2, num_novelty_inf))
                print("Number of states: novelty 1: {}; novelty 2: {}; novelty inf: {}.".format(num_novelty_1, num_novelty_2, num_novelty_inf))
                return novel_pairs
            else:
                return None

        iteration += 1
        # logging.debug(
        #     "novelty_search: Iteration %d, #unexplored=%d"
        #     % (iteration, len(priority_queue))
        # )
        # get the next node to explore
        # node = queue.popleft()
        _, _, node = heapq.heappop(priority_queue)

        # if all:
        #     all_pairs+=node.extract_state_value_pairs(distance=distance)
        # if novel:
            

        # exploring the node or if it is a goal node extracting the plan
        if planning_task.goal_reached(node.state):
            logging.info("Goal reached. Start extraction of solution.")
            logging.info("%d Nodes expanded" % iteration)
            logging.info("search_time: %d" % (time.perf_counter() - start_time))
            if all:
            #     return all_pairs
            # if novel:
                # file.write("total number of states: {}; novelty 1: {}; novelty 2: {}; nonnovel: {}.\n".format(
                #     num_novelty_1+num_novelty_2+num_novelty_inf, num_novelty_1, num_novelty_2, num_novelty_inf))
                print("Number of states: novelty 1: {}; novelty 2: {}; novelty inf: {}.".format(num_novelty_1, num_novelty_2, num_novelty_inf))
                return novel_pairs
            else:
                return node.extract_state_value_pairs(distance=distance)
                
        for operator, successor_state in planning_task.get_successor_states(node.state):
            # duplicate detection
            if successor_state not in closed:
                # tuples are updated in place, thus novelty computation is affected by the order
                single_tuples, double_tuples, novelty, novel_set = searchspace.compute_novelty(single_tuples, double_tuples, successor_state)
                node.novelty = novelty
                node.novel_set = novel_set
                if novel==0:
                    # novel=0 means no requirement on novelty
                    num_novelty_inf+=1
                    novel_pairs+=node.extract_state_value_pairs(distance=distance, novel=novel, lifted=lifted)
                    node_tiebreaker += 1
                    succ_node = searchspace.make_child_node(node, operator, successor_state)
                    heapq.heappush(priority_queue, (0, node_tiebreaker,succ_node)) 

                elif novelty<=novel and novelty==1:
                    num_novelty_1+=1
                    novel_pairs+=node.extract_state_value_pairs(distance=distance, novel=novel, lifted=lifted)
                    node_tiebreaker += 1
                    succ_node = searchspace.make_child_node(node, operator, successor_state)
                    heapq.heappush(priority_queue, (novelty, node_tiebreaker,succ_node))

                elif novelty<=novel and novelty==2:
                    num_novelty_2+=1
                    novel_pairs+=node.extract_state_value_pairs(distance=distance, novel=novel, lifted=lifted)
                    node_tiebreaker += 1
                    succ_node = searchspace.make_child_node(node, operator, successor_state)
                    heapq.heappush(priority_queue, (novelty, node_tiebreaker,succ_node))

                else:
                    num_novelty_inf+=1
                    # node_tiebreaker += 1
                    # succ_node = searchspace.make_child_node(node, operator, successor_state)
                    # heapq.heappush(priority_queue, (float('inf'), node_tiebreaker,succ_node))
                
                # remember the successor state
                closed.add(successor_state)
    logging.info("No operators left. Task unsolvable.")
    logging.info("%d Nodes expanded" % iteration)
    if all:
        return novel_pairs
    return None
