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
In search.py, we implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from math import sqrt
import util
import heapq
from collections import defaultdict, deque

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
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
    return  [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    frontier = util.Stack()
    exploredNodes = set()
    frontierStates = set() 
    startState = problem.getStartState()
    startNode = (startState, [])
    fringe_size = 0

    frontier.push(startNode)
    frontierStates.add(startState)  

    while not frontier.isEmpty():
        currentState, actions = frontier.pop()
        frontierStates.remove(currentState)  
        fringe_size = max(fringe_size, len(frontierStates) + len(exploredNodes))

        if currentState not in exploredNodes:
            exploredNodes.add(currentState)
            if problem.isGoalState(currentState):
                return actions, len(exploredNodes), len(actions), fringe_size
            else:
                for succState, succAction, _ in problem.getSuccessors(currentState):
                    if succState not in exploredNodes and succState not in frontierStates:
                        newAction = actions + [succAction]
                        frontier.push((succState, newAction))
                        frontierStates.add(succState) 

    return [], len(exploredNodes), 0, fringe_size


def breadthFirstSearch(problem):
    exploredNodes = set()
    frontier = deque()  # Use deque for efficient FIFO queue
    startState = problem.getStartState()
    startNode = (startState, [])
    fringe_size = 0
    expanded_nodes = 0

    if problem.isGoalState(startState):
        return [], expanded_nodes, 0, fringe_size  # If start is goal

    frontier.append(startNode)
    exploredNodes.add(startState)

    while frontier:
        currentState, actions = frontier.popleft()
        fringe_size = max(fringe_size, len(frontier) + len(exploredNodes))

        expanded_nodes += 1
        for succState, succAction, _ in problem.getSuccessors(currentState):
            if succState not in exploredNodes:
                if problem.isGoalState(succState):
                    return actions + [succAction], expanded_nodes, len(actions) + 1, fringe_size
                exploredNodes.add(succState)
                newActions = actions + [succAction]
                frontier.append((succState, newActions))

    return [], expanded_nodes, 0, fringe_size


        

def uniformCostSearch(problem):
    fringe_size = 0
    expanded_nodes = 0

    # Initialize the priority queue
    frontier = []
    heapq.heappush(frontier, (0, 0, problem.getStartState(), []))  # (cost, count, state, actions)
    explored = set()
    costs = defaultdict(lambda: float('inf'))
    costs[problem.getStartState()] = 0

    count = 0  # Unique sequence count

    while frontier:
        currentCost, _, currentState, actions = heapq.heappop(frontier)

        if currentState in explored:
            continue
        explored.add(currentState)

        if problem.isGoalState(currentState):
            return actions, expanded_nodes, len(actions), max(fringe_size, len(frontier))

        expanded_nodes += 1

        for succState, succAction, succCost in problem.getSuccessors(currentState):
            newCost = currentCost + succCost
            if newCost < costs[succState]:
                count += 1
                heapq.heappush(frontier, (newCost, count, succState, actions + [succAction]))
                costs[succState] = newCost

        # Update fringe size after potential expansions
        fringe_size = max(fringe_size, len(frontier))

    return [], expanded_nodes, 0, fringe_size




def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def h1(state, problem=None):
    heuristic = 0
    goal_state = [[0, 1, 2], [3, 4, 5], [6 , 7, 8]]
    for i in range(3):
        for j in range(3):
            if(state.cells[i][j] != goal_state[i][j] and state.cells[i][j]!=0):
                heuristic+=1
    return heuristic

def find_goal_tile(tile, goal_tile):
    for i in range(3):
        for j in range(3): 
            if goal_tile[i][j] == tile:
                return (i, j)

def h2(state, problem=None):
    heuristic = 0
    goal_state = [[0, 1, 2], [3, 4, 5], [6 , 7, 8]]

    actual_state = state.cells[:]

    for i in range(3):
        for j in range(3):
            if(state.cells[i][j]!=0): 
                goal_tile = find_goal_tile(state.cells[i][j], goal_state)
                heuristic += sqrt( (goal_tile[0] - i)**2  +  (goal_tile[1] - j)**2 )

    return heuristic

def h3(state, problem=None):
    heuristic = 0
    goal_state = [[0, 1, 2], [3, 4, 5], [6 , 7, 8]]

    for i in range(3):
        for j in range(3):
            if(state.cells[i][j]!=0): 
                goal_tile = find_goal_tile(state.cells[i][j], goal_state)
                heuristic += abs(goal_tile[0] - i) + abs(goal_tile[1] - j)
    return heuristic
   
def h4(state, problem=None):
    heuristic = 0
    goal_state = [[0, 1, 2], [3, 4, 5], [6 , 7, 8]]

    for i in range(3):
        for j in range(3):
            if(state.cells[i][j]//3!=i and state.cells[i][j]!=0):
                heuristic +=1
            if(state.cells[i][j]%3!=i and state.cells[i][j]!=0):
                heuristic +=1
    
    return heuristic

    

def aStarSearch(problem, heuristic):

    frontier = util.PriorityQueue()
    fringe_size = 0
    exploredNodes = set()  # Changed to set for O(1) lookup
    startState = problem.getStartState()
    startNode = (startState, [], 0)  # (state, action, cost)

    frontier.push(startNode, 0)
    # Track costs to reach states that have been seen
    seenStates = {startState: 0}

    while not frontier.isEmpty():
        currentState, actions, currentCost = frontier.pop()

        if currentState in exploredNodes:
            continue

        exploredNodes.add(currentState)
        fringe_size = max(frontier.count, fringe_size)  

        if problem.isGoalState(currentState):
            return actions, len(exploredNodes), len(actions), fringe_size

        for succState, succAction, succCost in problem.getSuccessors(currentState):
            newAction = actions + [succAction]
            newCost = currentCost + succCost

            # Only process new state or better cost found
            if succState not in seenStates or newCost < seenStates[succState]:
                seenStates[succState] = newCost
                frontier.push((succState, newAction, newCost), newCost + heuristic(succState, problem))

    return [], len(exploredNodes), 0, fringe_size  # Return empty path if not found


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

