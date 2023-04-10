# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


"""
def DFS(problem: SearchProblem, state, state_visited, actions):
    if state in state_visited:
        return False
    state_visited.append(state)
    if problem.isGoalState(state):
        return True
    successors = problem.getSuccessors(state)
    for successor in successors:
        next_state = successor[0]
        if DFS(problem, next_state, state_visited, actions):
            actions.append(successor[1])
            return True
    return False
"""


# dfs和bfs实现方式大致相同，只有数据结构不同，dfs是栈，bfs是队列
def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    """
    state_visited = []
    actions = []
    start_state = problem.getStartState()

    if DFS(problem, start_state, state_visited, actions):
        actions.reverse()
        return actions
    else:
        return False
    """
    closed = []  # closed表，用于存放从栈中弹出的数据
    state_visited = [problem.getStartState()]  # 记录已被访问的结点
    state_stack = util.Stack()  # 存放状态的栈
    state_stack.push(problem.getStartState())
    actions_pre = {problem.getStartState(): []}  # 字典形式，存放到达该状态之前的动作
    while not state_stack.isEmpty():  # dfs:每次弹出栈顶状态，将栈顶状态可到达的状态均压入栈中，直至到达最终状态
        state = state_stack.pop()
        if problem.isGoalState(state):  # 到达最终状态，输出actions序列
            return actions_pre[state]
        if state not in closed:
            closed.append(state)
            successors = problem.getSuccessors(state)
            for successor in successors:
                next_state = successor[0]
                if next_state in state_visited:
                    actions_pre[next_state] = actions_pre[state] + [successor[1]]
                    continue
                actions_pre[next_state] = actions_pre[state] + [successor[1]]
                state_stack.push(next_state)
                state_visited.append(next_state)
    return False


# bfs与dfs大致相同，数据结构为队列
def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    closed = []
    state_visited = [problem.getStartState()]
    state_queue = util.Queue()
    state_queue.push(problem.getStartState())
    actions_pre = {problem.getStartState(): []}
    while not state_queue.isEmpty():
        state = state_queue.pop()
        if problem.isGoalState(state):
            return actions_pre[state]
        if state not in closed:
            closed.append(state)
            successors = problem.getSuccessors(state)
            for successor in successors:
                next_state = successor[0]
                if next_state in state_visited:
                    continue
                actions_pre[next_state] = actions_pre[state] + [successor[1]]
                state_queue.push(next_state)
                state_visited.append(next_state)
    return False


# ufs和astar实现方式大致相同，数据结构均为优先级队列，只是cost代价表示方式不同
def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    closed = []
    start_state = problem.getStartState()
    cost = {start_state: 0}                                         # cost字典，用于存放从起点到达每一点的代价
    action_pre = {start_state: []}
    state_visited = [start_state]
    state_priorityqueue = util.PriorityQueue()
    state_priorityqueue.push(start_state, 0)
    while not state_priorityqueue.isEmpty():
        state = state_priorityqueue.pop()
        if problem.isGoalState(state):
            return action_pre[state]
        if state not in closed:
            closed.append(state)
            successors = problem.getSuccessors(state)
            for successor in successors:
                next_state = successor[0]
                new_actions = action_pre[state] + [successor[1]]
                if next_state in state_visited:                                         # 无论该点之前是否被访问过，均需更新该点的代价
                    if cost[next_state] > problem.getCostOfActions(new_actions):
                        action_pre[next_state] = new_actions
                        cost[next_state] = problem.getCostOfActions(action_pre[next_state])
                    continue
                action_pre[next_state] = new_actions
                cost[next_state] = problem.getCostOfActions(action_pre[next_state])
                state_priorityqueue.update(next_state, cost[next_state])
                state_visited.append(next_state)
    return False


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    closed = []
    start_state = problem.getStartState()
    cost = {start_state: 0}
    action_pre = {start_state: []}
    state_visited = [start_state]
    state_priorityqueue = util.PriorityQueue()
    state_priorityqueue.push(start_state, 0)
    while not state_priorityqueue.isEmpty():
        state = state_priorityqueue.pop()
        if problem.isGoalState(state):
            return action_pre[state]
        if state not in closed:
            closed.append(state)
            successors = problem.getSuccessors(state)
            for successor in successors:
                next_state = successor[0]
                new_actions = action_pre[state] + [successor[1]]
                new_cost = problem.getCostOfActions(new_actions) + heuristic(next_state, problem)       # astar与ufs不同点仅为代价函数不同，需增加启发式函数
                if next_state in state_visited:                                                         # 无论该点之前是否被访问过，均需更新该点的代价
                    if cost[next_state] > new_cost:
                        action_pre[next_state] = new_actions
                        cost[next_state] = new_cost
                        state_priorityqueue.update(next_state, cost[next_state])
                    continue
                action_pre[next_state] = new_actions
                cost[next_state] = new_cost
                state_priorityqueue.update(next_state, cost[next_state])
                state_visited.append(next_state)
    return False


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
