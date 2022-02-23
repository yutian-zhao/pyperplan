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
Building the search node and associated methods
"""


class SearchNode:
    """
    The SearchNode class implements recursive data structure to build a
    search space for planning algorithms. Each node links to is parent
    node and contains informations about the state, action to arrive
    the node and the path length in the count of applied operators.
    """

    def __init__(self, state, parent, action, g, novel_parent=None):
        """
        Construct a search node

        @param state: The state to store in the search space.
        @param parent: The parent node in the search space.
        @param action: The action which produced the state.
        @param g: The path length of the node in the count of applied
                  operators.
        """
        self.state = state
        self.parent = parent
        self.action = action
        self.g = g
        self.novelty = None
        self.novel_set = None

    def extract_solution(self):
        """
        Returns the list of actions that were applied from the initial node to
        the goal node.
        """
        solution = []
        while self.parent is not None:
            solution.append(self.action)
            self = self.parent
        solution.reverse()
        return solution

    def extract_state_value_pairs(self, distance=0, novel=0, lifted=0):
        """
        Returns the list of actions that were applied from the initial node to
        the goal node.
        """
        if novel and self.novelty>novel:
            return []

        if lifted and novel:
            goal_states = [self.novel_set]
            lifted_state = set(self.novel_set)
            lifted -= 1
            while lifted>0 and len(lifted_state)<len(self.state):
                for atom in self.state:
                    if atom not in lifted_state:
                        lifted_state.add(atom)
                        goal_states.append(frozenset(lifted_state))
                        lifted -= 1
        else:
            goal_states = [self.state]

        state_value_pairs = []
        value = 0
        g = self.g
        while self is not None:
            # novel start
            if ((novel and self.novelty<=novel) or not novel) and (value>=distance):
                for goal_state in goal_states:
                    state_value_pairs.append((self.state, goal_state, value))
            value += 1
            self = self.parent
        state_value_pairs.reverse()
        # print('{}/{}={}'.format(len(state_value_pairs), g+1, len(state_value_pairs)/(g+1)))
        return state_value_pairs


def make_root_node(initial_state):
    """
    Construct an initial search node. The root node of the search space
    does not links to a parent node, does not contains an action and the
    g-value is zero.

    @param initial_state: The initial state of the search space.
    """
    return SearchNode(initial_state, None, None, 0)


def make_child_node(parent_node, action, state):
    """
    Construct a new search node containing the state and the applied action.
    The node is linked to the given parent node.
    The g-value is set to the parents g-value + 1.
    """
    return SearchNode(state, parent_node, action, parent_node.g + 1)

def compute_novelty(single_tuples, double_tuples, state):
    """
    This function only compute novelty up to 2. 

    @param single_tuples: frozen set of previous visited facts.
    @param double_tuples: frozen set of previous visited tuples of facts of length 2.
    @param state: the state to be compute novelty of.
    """
    flag1 = False # flag of novelty 1
    flag2 = False # flag of novelty 1
    novel_atoms = set()
    novel_atom_tuples = set()

    for atom in state:
        if atom not in single_tuples:
            single_tuples.add(atom)
            novel_atoms.add(atom)
            if not flag1:
                flag1 = True

    for atom1 in state:
        for atom2 in state:
            if atom1 != atom2:
                double_tuple = frozenset([atom1, atom2])
                if double_tuple not in double_tuples:
                    double_tuples.add(double_tuple)
                    for i in double_tuple:
                        novel_atom_tuples.add(i)
                    if not flag2:
                        flag2 = True
    
    if flag1:
        return single_tuples, double_tuples, 1, frozenset(novel_atoms)
    elif flag2:
        return single_tuples, double_tuples, 2, frozenset(novel_atom_tuples)
    else:
        return single_tuples, double_tuples, float('inf'), None

