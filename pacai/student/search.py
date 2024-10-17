from pacai.util.stack import Stack
from pacai.util.queue import Queue
from pacai.util.priorityQueue import PriorityQueue

"""
In this file, you will implement generic search algorithms which are called by Pacman agents.
"""

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first [p 85].

    Your search algorithm needs to return a list of actions that reaches the goal.
    Make sure to implement a graph search algorithm [Fig. 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    ```
    print("Start: %s" % (str(problem.startingState())))
    print("Is the start a goal?: %s" % (problem.isGoal(problem.startingState())))
    print("Start's successors: %s" % (problem.successorStates(problem.startingState())))
    ```
    """

    fringe = Stack()
    visited = set()

    fringe.push((problem.startingState(), [], 0))

    while not fringe.isEmpty():
        state, actions, cost = fringe.pop()

        if problem.isGoal(state):
            return actions
        
        if state not in visited:
            visited.add(state)

            for successor, action, step_cost in problem.successorStates(state):
                fringe.push((successor, actions + [action], cost + step_cost))
    
    return []

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first. [p 81]
    """

    fringe = Queue()
    visited = set()

    fringe.push((problem.startingState(), []))

    while not fringe.isEmpty():
        state, actions = fringe.pop()

        if problem.isGoal(state):
            return actions

        if state not in visited:
            visited.add(state)

            for successor, action, step_cost in problem.successorStates(state):
                fringe.push((successor, actions + [action]))
    
    return []

def uniformCostSearch(problem):
    """
    Search the node of least total cost first.
    """
    fringe = PriorityQueue()
    visited = set()

    fringe.push((problem.startingState(), [], 0), 0)

    while not fringe.isEmpty():
        state, actions, cost = fringe.pop()

        if problem.isGoal(state):
            return actions

        if state not in visited:
            visited.add(state)

            for successor, action, step_cost in problem.successorStates(state):
                new_cost = cost + step_cost
                fringe.push((successor, actions + [action], new_cost), new_cost)
    
    return []

def aStarSearch(problem, heuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    fringe = PriorityQueue()
    explored = {}

    start_state = problem.startingState()
    
    fringe.push((start_state, [], 0), heuristic(start_state, problem))

    while not fringe.isEmpty():
        state, actions, cost = fringe.pop()

        if problem.isGoal(state):
            return actions

        if state in explored and explored[state] <= cost:
            continue

        explored[state] = cost

        for successor, action, step_cost in problem.successorStates(state):
            new_cost = cost + step_cost
            priority = new_cost + heuristic(successor, problem)

            if successor not in explored or new_cost < explored.get(successor, float('inf')):
                fringe.push((successor, actions + [action], new_cost), priority)
    return []