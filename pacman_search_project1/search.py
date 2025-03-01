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
from game import Directions
from typing import List

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




def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
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
    # defining visited set (as dict), the stack, the parent nodes (immediate prev node), and solution path 
    visited = {}
    stack = util.Stack()
    parentNodes = {} 
    solution = []

    # start state gets pushed onto the stack
    stack.push((problem.getStartState(), 'N/A', 0))
    visited[problem.getStartState()] = 'N/A'

    # if start is the goal state, return solution path
    if problem.isGoalState(problem.getStartState()):
        return solution
    
    # loop to go through all nodes and find the optimal path until goal state is reached
    goalReached = False
    while not stack.isEmpty() and not goalReached:
        state = stack.pop()
        visited[state[0]] = state[1]
        
        # if vertex is the goal state, return solution path
        if problem.isGoalState(state[0]):
            vertex = state[0]
            goalReached = True
            break

        # adding adjacent values if not visited
        for successorTriple in problem.getSuccessors(state[0]):
            if successorTriple[0] not in visited.keys():
                parentNodes[successorTriple[0]] = state[0]
                stack.push(successorTriple)  
    
    # loop to store the path
    while(vertex in parentNodes.keys()):
        prev_vert = parentNodes[vertex]
        solution.insert(0, visited[vertex])
        vertex = prev_vert
    
    return solution
    util.raiseNotDefined()


def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # defining visited set (as dict), the queue, the parent nodes (as dict), and solution path 
    visited = {}
    queue = util.Queue()
    parentNodes = {}
    solution = []

    # start state gets pushed onto the queue
    queue.push((problem.getStartState(), 'N/A', 0))
    visited[problem.getStartState()] = 'N/A'

    # if start is the goal state, return solution path
    if problem.isGoalState(problem.getStartState()):
        return solution
    
    # loop to go through all nodes and find the optimal path until goal state is reached
    goalReached = False
    while not queue.isEmpty() and not goalReached:
        state = queue.pop()
        visited[state[0]] = state[1]
        
        # if vertex is the goal state, return solution path
        if problem.isGoalState(state[0]):
            vertex = state[0]
            goalReached = True
            break

        # adding adjacent values if not visited or expanded by another vertex
        for successorTriple in problem.getSuccessors(state[0]):
            if successorTriple[0] not in visited.keys() and successorTriple[0] not in parentNodes.keys():
                parentNodes[successorTriple[0]] = state[0]
                queue.push(successorTriple)  
    
    # loop to store the path
    while(vertex in parentNodes.keys()):
        prev_vert = parentNodes[vertex]
        solution.insert(0, visited[vertex])
        vertex = prev_vert
    
    return solution
    util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # defining visited set (as dict), the queue, the parent nodes (as dict), solution path, and cost weights
    visited = {}
    pQueue = util.PriorityQueue()
    parentNodes = {}
    solution = []
    stepCosts = {}

    # start state gets pushed onto the queue
    pQueue.push((problem.getStartState(), 'N/A', 0), 0)
    visited[problem.getStartState()] = 'N/A'
    stepCosts[problem.getStartState()] = 0

    # if start is the goal state, return solution path
    if problem.isGoalState(problem.getStartState()):
        return solution
    
    # loop to go through all nodes and find the optimal path until goal state is reached
    goalReached = False
    while not pQueue.isEmpty() and not goalReached:
        state = pQueue.pop()
        visited[state[0]] = state[1]
        
        # if vertex is the goal state, return solution path
        if problem.isGoalState(state[0]):
            vertex = state[0]
            goalReached = True
            break

        # adding adjacent values if not visited or expanded by another vertex
        for successorTriple in problem.getSuccessors(state[0]):
            if successorTriple[0] not in visited.keys():
                # calc the new weight/cost of the path
                newCost = state[2] + successorTriple[2]
                # the new cost is less than the old one or it just hasn't been visited yet, visit
                if (successorTriple[0] not in stepCosts.keys()) or (stepCosts[successorTriple[0]] > newCost):
                    pQueue.push((successorTriple[0], successorTriple[1], newCost), newCost)
                    stepCosts[successorTriple[0]] = newCost
                    parentNodes[successorTriple[0]] = state[0]
    
    # loop to store the path
    while(vertex in parentNodes.keys()):
        prev_vert = parentNodes[vertex]
        solution.insert(0, visited[vertex])
        vertex = prev_vert
    
    return solution
    util.raiseNotDefined()

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # defining visited set (as dict), the queue, the parent nodes (as dict), solution path, and cost weights
    visited = {}
    pQueue = util.PriorityQueue()
    parentNodes = {}
    solution = []
    stepCosts = {}

    # start state gets pushed onto the queue
    pQueue.push((problem.getStartState(), 'N/A', 0), 0)
    visited[problem.getStartState()] = 'N/A'
    stepCosts[problem.getStartState()] = 0

    # if start is the goal state, return solution path
    if problem.isGoalState(problem.getStartState()):
        return solution
    
    # loop to go through all nodes and find the optimal path until goal state is reached
    goalReached = False
    while not pQueue.isEmpty() and not goalReached:
        state = pQueue.pop()
        visited[state[0]] = state[1]
        
        # if vertex is the goal state, return solution path
        if problem.isGoalState(state[0]):
            vertex = state[0]
            goalReached = True
            break

        # adding adjacent values if not visited or expanded by another vertex
        for successorTriple in problem.getSuccessors(state[0]):
            if successorTriple[0] not in visited.keys():
                # calc the new weight/cost of the path w the heuristic of the state
                newCost = state[2] + successorTriple[2] + heuristic(successorTriple[0], problem)
                # the new cost is less than the old one or it just hasn't been visited yet, visit
                if (successorTriple[0] not in stepCosts.keys()) or (stepCosts[successorTriple[0]] > newCost):
                    pQueue.push((successorTriple[0], successorTriple[1], state[2] + successorTriple[2]), newCost)
                    stepCosts[successorTriple[0]] = newCost
                    parentNodes[successorTriple[0]] = state[0]
    
    # loop to store the path
    while(vertex in parentNodes.keys()):
        prev_vert = parentNodes[vertex]
        solution.insert(0, visited[vertex])
        vertex = prev_vert
    
    return solution
    util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
