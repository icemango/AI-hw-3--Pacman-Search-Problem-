# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
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
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 85].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:

  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())

  
  Get action command
  """
  from game import Directions  
  s = Directions.SOUTH
  w = Directions.WEST
  n = Directions.NORTH
  e = Directions.EAST

  """
  Do DFS
  """
  from util import Stack
  stack=Stack()
  path=[]
  action=[]
  goneNode = {'<0,0>':'1'}
  #1==>has gone
  currentStateIdx=problem.getStartState()
  startState=currentStateIdx
  totalPath=-1
  while problem.isGoalState(currentStateIdx)== False:
    nextState=problem.getSuccessors(currentStateIdx)
    state=nextState[0]
    for state in nextState:
      stack.push(state)

    currentState=stack.pop()
    currentStateIdx=currentState[0]
    #print currentState
    #print problem.isGoalState(currentStateIdx)
    
    while currentStateIdx in goneNode and stack.isEmpty() == False:
      currentState=stack.pop()
      currentStateIdx=currentState[0]
      
    path.append(currentState)
    totalPath=totalPath+1
    goneNode[currentStateIdx]=1;

  """
  DFS Done
  """

  """
  Trace the current path
  """

  currentState=path[totalPath]
  if currentState[1] == "North":
    action.append(n)
  if currentState[1] == "South":
    action.append(s)
  if currentState[1] == "West":
    action.append(w)
  if currentState[1] == "East":
    action.append(e)
    
  currentStateIdx = currentState[0]
  beforeState=[]
  while currentStateIdx != startState:
    #print currentState
    #print beforeState;
    #print "=============="
    if currentState == beforeState:
      break;
    beforeState=currentState
    i=0
    flag=0
    while i < totalPath:
      tmpState=path[i]
      i=i+1
      nearState=problem.getSuccessors(tmpState[0])
      #print nearState
      for state in nearState:             #find the connect state
        #print state
        #print currentState
        #print "===================="
        if state == currentState:
          #print state
          #print currentState
          #print "============"
          currentState=tmpState
          currentStateIdx=currentState[0]
          if currentStateIdx != startState:
            if currentState[1] == "North":
               action.append(n)
            if currentState[1] == "South":
               action.append(s)
            if currentState[1] == "West":
               action.append(w)
            if currentState[1] == "East":
               action.append(e)
          flag=1;
      if flag != 0:
        break

          
  action.reverse()
  #print action
  return action

  util.raiseNotDefined()

def breadthFirstSearch(problem):
    "Search the shallowest nodes in the search tree first. [p 81]"
    "*** YOUR CODE HERE ***"
    path = { problem.getStartState():(None,None) }
    visited={}
    def visit(state,parent) :
        if state[0] not in path :
            path[state[0]] = ( parent,state[1] )
    bfsQ = util.Queue()
    bfsQ.push( problem.getStartState() )
    while not bfsQ.isEmpty() :
        V = bfsQ.pop()
        visited[V]=True
        if problem.isGoalState(V):   #goal & retrieve path
            result=[]
            traversal=V
            while path[traversal][0]!=None:
                result.append(path[traversal][1])
                traversal=path[traversal][0]
            result.reverse()
            return result
        for W in problem.getSuccessors(V):
            if not visited.get(W[0],0) :
                visit(W,V)
                bfsQ.push(W[0])
    return None
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "

  "Search the shallowest nodes in the search tree first. [p 81]"
  "*** YOUR CODE HERE ***"
  path = { problem.getStartState():(None,None) }
  priority = 1
  visited={}
  def visit(state,parent) :
      if state[0] not in path :
          path[state[0]] = ( parent,state[1] )
  bfsQ = util.PriorityQueue()
  #print problem.getStartState()
  bfsQ.push( problem.getStartState(),1 )
  while not bfsQ.isEmpty() :
      V = bfsQ.pop()
      
      tmp2 = problem.getSuccessors(V)
      tmp =  problem.getSuccessors(tmp2[0][0])
      state = []
      for state in tmp:
        if state[0] == V:
          priority = priority + state[2]
      
      visited[V]=True
      if problem.isGoalState(V):   #goal & retrieve path
          result=[]
          traversal=V
          while path[traversal][0]!=None:
              result.append(path[traversal][1])
              traversal=path[traversal][0]
          result.reverse()
          return result
      for W in problem.getSuccessors(V):
          if not visited.get(W[0],0) :
              visit(W,V)
              bfsQ.push(W[0],W[2]+priority)
  return None



def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    path = {}   #for path retrieve
    F={}
    visited={}  #closed list
    priorityQ = util.PriorityQueue()
    V=problem.getStartState()
    path[V]=(None,None)
    F[V]=heuristic(V,problem)
    priorityQ.push(V,F[V])
    open={V:True}
    while not priorityQ.isEmpty():
        V=priorityQ.pop()
        visited[V]=True
        open[V]=False
        if problem.isGoalState(V):  #Goal & retrieve path
            result=[]
            traversal=V
            while path[traversal][0]!=None:
                result.append(path[traversal][1])
                traversal=path[traversal][0]
            result.reverse()
            return result
        for W in problem.getSuccessors(V):
            F_W=F[V]-heuristic(V,problem)+heuristic(W[0],problem)+1
            if visited.get(W[0],False):
                if F[W[0]]>F_W :
                    path[W[0]]=(V,W[1])
                    F[W[0]]=F_W
                    priorityQ.push(W[0],F_W)
                    visited[W[0]]=False
                    open[W[0]]=True
            else:
                if open.get(W[0],False):
                    if F[W[0]] > F_W :
                        path[W[0]]=(V,W[1])
                        F[W[0]]=F_W
                        priorityQ.push(W[0],F_W)
                        open[W[0]]=True
                else:
                    F[W[0]]=F_W
                    path[W[0]]=(V,W[1])
                    priorityQ.push(W[0],F_W)
                    open[W[0]]=True
    return None

  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
