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
import searchAgents

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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    visited  = []
    stack = util.Stack()
    stack.push((problem.getStartState(), []))
    while stack:
        (state, path) = stack.pop()
        if problem.isGoalState(state):
            return path
        visited.append(state)
        for succ, action, cost in problem.getSuccessors(state):
            if succ not in visited:
                stack.push((succ, path + [action]))
    return []
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    startState = problem.getStartState()
    visited = [startState]
    from util import Queue
    q = Queue()
    q.push((startState,[]))
    while not q.isEmpty():
        (currentState, direction) = q.pop()
        isGoal = problem.isGoalState(currentState)
        if isGoal == 'Intermediate':
            #print 'intr ', currentState
            while not q.isEmpty():
                q.pop()
            visited = []
        elif isGoal:
            #print 'goal'
            return direction
        for succ, dirc, cost in problem.getSuccessors(currentState): 
            if succ not in visited:
                q.push((succ, direction + [dirc]))
                visited.append(succ)
    return direction
   
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    directions = []
    startState = problem.getStartState()
    visited = [startState]
    q = util.PriorityQueue()  #same as bfs but use priority queue
    q.push(startState, 0)
    predMap = {startState:(None,None,0)} # key is state, value is a tuple (parent, action, cost)
    while not q.isEmpty():
        currentState = q.pop()
        (parent, direction, currCost) = predMap[currentState]
        visited.append(currentState)
        if problem.isGoalState(currentState):
            while parent:
                directions.insert(0, direction)
                (parent, direction, currCost) = predMap[parent]
            return directions
        for succ, direction, cost in problem.getSuccessors(currentState): 
            if succ not in visited:
                q.update(succ, cost+currCost)
                if succ in predMap:
                    succParent, succDir, succCost = predMap[succ]
                    if succCost > cost+currCost:
                       predMap[succ] = (currentState, direction, cost+currCost)                        
                else:
                    predMap[succ] = (currentState, direction, cost+currCost) 
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    directions = []
    startState = problem.getStartState()
    visited = [startState]
    q = util.PriorityQueue()  #same as bfs but use priority queue
    q.push(startState, 0)
    predMap = {startState:(None,None,0)} # key is state, value is a tuple (parent, action, cost)
    while not q.isEmpty():
        currentState = q.pop()
        (parent, direction, currCost) = predMap[currentState]
        visited.append(currentState)
        if problem.isGoalState(currentState):
            while parent:
                directions.insert(0, direction)
                (parent, direction, currCost) = predMap[parent]
            return directions
        for succ, direction, cost in problem.getSuccessors(currentState):
            if succ not in visited:
                newCost = cost + currCost + heuristic(succ,problem)
                q.update(succ, newCost)
                if succ in predMap:
                    (succParent, succDir, succCost) = predMap[succ]
                    if succCost + heuristic(succ,problem) > newCost:
                        predMap[succ] = (currentState, direction, cost + currCost)
                else:
                    predMap[succ] = (currentState, direction, cost + currCost)
            
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
