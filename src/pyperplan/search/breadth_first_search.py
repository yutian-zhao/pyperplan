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
Implements the breadth first search algorithm.
"""

from collections import deque
import logging

from pyperplan.search import searchspace

# add time limit
import time
import os # for debugging


def breadth_first_search(planning_task, max_search_time=float("inf"), mode=None):
    """
    Searches for a plan on the given task using breadth first search and
    duplicate detection.

    @param planning_task: The planning task to solve.
    @return: The solution as a list of operators or None if the task is
    unsolvable.
    """
    # counts the number of loops (only for printing)
    iteration = 0
    # fifo-queue storing the nodes which are next to explore
    queue = deque()
    queue.append(searchspace.make_root_node(planning_task.initial_state))
    # set storing the explored nodes, used for duplicate detection
    closed = {planning_task.initial_state}

    all = mode.get('all', False) if mode else False
    novel = mode.get('novel', 0) if mode else 0
    distance = mode.get('distance', 0) if mode else 0
    lifted = mode.get('lifted', False) if mode else False
    _log.info("Mode: {}".format(mode))

    if all:
        all_pairs = []
    elif novel:
        # file = open(os.path.join(os.getcwd(), "novelty.log"), 'a')
        # file.write("total number of facts of {}: {}. (n*(n-1)={}).\n".format(planning_task.name, len(planning_task.facts), len(planning_task.facts)*(len(planning_task.facts)-1)))
        novel_pairs = []
        num_novelty_1 = 0
        num_novelty_2 = 0
        num_novelty_inf = 0
        single_tuples = set()
        double_tuples = set()

    start_time = time.perf_counter()

    while queue:
        elapsed_time = time.perf_counter() - start_time
        if elapsed_time >= max_search_time:
            logging.info("Search timed out")
            logging.info("search_time: %d" % elapsed_time)
            if all:
                return all_pairs
            elif novel:
                logging.info("Number of states: {}; novelty 1: {}; novelty 2: {}; novelty inf: {}.".format( 
                    (num_novelty_1+num_novelty_2+num_novelty_inf),num_novelty_1, num_novelty_2, num_novelty_inf))
                return novel_pairs
            else:
                return None

        iteration += 1
        # logging.debug(
        #     "breadth_first_search: Iteration %d, #unexplored=%d"
        #     % (iteration, len(queue))
        # )
        # get the next node to explore
        node = queue.popleft()

        # exploring the node or if it is a goal node extracting the plan
        if planning_task.goal_reached(node.state):
            logging.info("Goal reached. Start extraction of solution.")
            logging.info("%d Nodes expanded" % iteration)
            logging.info("search_time: %d" % (time.perf_counter() - start_time))
            if all:
                return all_pairs
            elif novel:
                logging.info("Number of states: {}; novelty 1: {}; novelty 2: {}; novelty inf: {}.".format( 
                    (num_novelty_1+num_novelty_2+num_novelty_inf),num_novelty_1, num_novelty_2, num_novelty_inf))
                return novel_pairs
            else:
                return node.extract_state_value_pairs(distance=distance)
                
        for operator, successor_state in planning_task.get_successor_states(node.state):
            # duplicate detection
            if successor_state not in closed:

                if all:
                    all_pairs+=node.extract_state_value_pairs(distance=distance)
                elif novel:
                    pop_state = node.state
                    single_tuples, double_tuples, novelty, novel_set = searchspace.compute_novelty(single_tuples, double_tuples, pop_state)
                    node.novelty = novelty
                    node.novel_set = novel_set
                    if novelty==1 and novelty<=novel:
                        num_novelty_1+=1
                        novel_pairs+=node.extract_state_value_pairs(distance=distance, novel=novel, lifted=lifted)
                    elif novelty==2 and novelty<=novel:
                        num_novelty_2+=1
                        novel_pairs+=node.extract_state_value_pairs(distance=distance, novel=novel, lifted=lifted)
                    else:
                        num_novelty_inf+=1

                queue.append(
                    searchspace.make_child_node(node, operator, successor_state)
                )
                # remember the successor state
                closed.add(successor_state)
    logging.info("No operators left. Task unsolvable.")
    logging.info("%d Nodes expanded" % iteration)
    
    if all:
        return all_pairs
    elif novel:
        logging.info("Number of states: {}; novelty 1: {}; novelty 2: {}; novelty inf: {}.".format( 
            (num_novelty_1+num_novelty_2+num_novelty_inf),num_novelty_1, num_novelty_2, num_novelty_inf))
        return novel_pairs
    else:
        return None
