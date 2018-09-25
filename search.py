# NOTE TO STUDENT: Please read the handout before continuing.

from tilegameproblem import TileGame
from dgraph import DGraph
from queue import Queue, LifoQueue, PriorityQueue

"""

implemented a dictonary of paths to check if the node has already been visited
and if the node is already in the queue. The paths dictionary also holds the
path to which the algorithm took to get there.

"""

def bfs(problem):
    """
    Implement breadth-first search.

    Input:
        problem - the problem on which the search is conducted, a SearchProblem

    Output: a list of states representing the path of the solution

    """
    start = problem.get_start_state()
    paths = dict()
    paths[start] = None
    q = Queue()
    q.put(start)

    while not q.empty():
        node = q.get()

        if problem.is_goal_state(node):
            result = list()
            result.append(node)
            while paths[node] != None :
                parent = paths[node]
                result.append(parent)
                node = parent
            result.reverse()
            return result

        for (child, cost) in problem.get_successors(node).items() :
            if not (child in paths.keys()) :
                paths[child] = node
                q.put(child)



def dfs(problem):
    """
    Implement depth-first search.

    Input:
        problem - the problem on which the search is conducted, a SearchProblem

    Output: a list of states representing the path of the solution

    """
    start = problem.get_start_state()
    paths = dict()
    paths[start] = None
    q = LifoQueue()
    q.put(start)

    while not q.empty():
        node = q.get()

        if problem.is_goal_state(node):
            result = list()
            result.append(node)
            while paths[node] != None :
                parent = paths[node]
                result.append(parent)
                node = parent
            result.reverse()
            return result

        for (child, cost) in problem.get_successors(node).items() :
            if not (child in paths.keys()) :
                paths[child] = node
                q.put(child)


def ids(problem):
    """
    Implement iterative deepening search.

    Input:
        problem - the problem on which the search is conducted, a SearchProblem

    Output: a list of states representing the path of the solution

    """
    depth = 1
    start = problem.get_start_state()
    result = list()

    while (depth > 0) :
        paths = dict()
        paths[start] = None
        node = ids_help(problem, start, paths, depth)
        if node != None:
            result.append(node)
            while paths[node] != None:
                parent = paths[node]
                result.append(parent)
                node = parent
            result.reverse()
            return result
        else :
            depth += 1


def ids_help(problem, node, paths, depth):
    """
    a recursive function that searches for the goal state until specified depth

    Input:
        problem, node, paths, depth

    Output: the goal node
    """
    if problem.is_goal_state(node):
        return node
    if depth == 0 :
        return None
    for (child, cost) in problem.get_successors(node).items():
        if not (child in paths.keys()):
            paths[child] = node
            child_node = ids_help(problem, child, paths, depth-1)
            if child_node != None:
                return child_node
    return None





def bds(problem, goal):
    """
    Implement bi-directional search.

    The input 'goal' is a goal state (not a search problem, just a state)
    from which to begin the search toward the start state.

    Assume that the input search problem can be thought of as
    an undirected graph. That is, all actions in the search problem
    are reversible.

    Input:
        problem - the problem on which the search is conducted, a SearchProblem

    Output: a list of states representing the path of the solution

    """
    start = problem.get_start_state()
    begin_q = Queue()
    end_q = Queue()
    begin_q.put(start)
    end_q.put(goal)
    begin_paths = dict()
    end_paths = dict()
    begin_paths[start] = None
    end_paths[goal] = None
    result_nodes = list()
    shortest = float('inf')
    result = list()
    state = True;

    while (not begin_q.empty()) and (not end_q.empty()):
        begin_node = begin_q.get()
        end_node = end_q.get()

        for (child, cost) in problem.get_successors(begin_node).items():
            if not(child in begin_paths.keys()):
                begin_paths[child] = begin_node
                begin_q.put(child)

        for (child, cost) in problem.get_successors(end_node).items():
            if not(child in end_paths.keys()):
                end_paths[child] = end_node
                end_q.put(child)

        for node in begin_paths.keys():
            if node in end_paths.keys():
                result_nodes.append(node)

        if (len(result_nodes) != 0):
        	for node in result_nodes:
        		path = list()
        		node_to_end = node
        		node_to_start = node

        		path.append(node_to_start)
        		while begin_paths[node_to_start] != None:
        			parent = begin_paths[node_to_start]
        			path.append(parent)
        			node_to_start = parent
        		path.reverse()
        		while end_paths[node_to_end] != None:
        			parent = end_paths[node_to_end]
        			path.append(parent)
        			node_to_end = parent

        		length = len(path)
        		if(length < shortest):
        			shortest = length
        			result = path
        	return result



def astar(problem, heur):
    """
    Implement A* search.

    The given heuristic function will take in a state of the search problem
    and produce a real number

    Your implementation should be able to work with any heuristic, heur
    that is for the given search problem (but, of course, without a
    guarantee of optimality if heur is not admissible).

    Input:
        problem - the problem on which the search is conducted, a SearchProblem

    Output: a list of states representing the path of the solution

    """
    start = problem.get_start_state()
    score = dict()
    paths = dict()
    visited = set()
    score[start] = 0
    paths[start] = None
    pq = PriorityQueue()
    pq.put(start, heur(start))

    while not pq.empty():
        node = pq.get()
        if problem.is_goal_state(node):
            result = list()
            result.append(node)
            while paths[node] != None:
                parent = paths[node]
                result.append(parent)
                node = parent
            result.reverse()
            return result
        visited.add(node)

        for (child, cost) in problem.get_successors(node).items():
        	if child not in visited:
	        	paths[child] = node
	        	score[child] = score[node] + cost
	        	pq.put(child, score[child] + heur(child))


### SPECIFIC TO THE TILEGAME PROBLEM


def tilegame_heuristic(state):
    """
    Produces a real number for the given tile game state representing
    an estimate of the cost to get to the goal state.

    Input:
        state - the tilegame state to evaluate. Consult handout for how the tilegame state is represented

    Output: an integer

    """
    heur = 0
    for row in range(3):
        tuples = state[row]
        for col in range(3):
            value = tuples[col]
            if value == 1 :
                heur += abs(0-row) + abs(0-col)
            if value == 2 :
                heur += abs(0-row) + abs(1-col)
            if value == 3 :
                heur += abs(0-row) + abs(2-col)
            if value == 4 :
                heur += abs(1-row) + abs(0-col)
            if value == 5 :
                heur += abs(1-row) + abs(1-col)
            if value == 6 :
                heur += abs(1-row) + abs(2-col)
            if value == 7 :
                heur += abs(2-row) + abs(0-col)
            if value == 8 :
                heur += abs(2-row) + abs(1-col)
            if value == 9 :
                heur += abs(2-row) + abs(2-col)
    return (heur / 2)



### YOUR SANDBOX ###


def main():
    """
    Do whatever you want in here; this is for you.
    An example below shows how your functions might be used.
    """
    # initialize a random 3x3 TileGame problem
    tg = TileGame(3)
    goal = ((1,2,3),(4,5,6),(7,8,9))
    # tg = TileGame(2)
    # goal = ((1,2),(3,4))
    print(TileGame.board_to_pretty_string(tg.get_start_state()))
    # compute path
    path = bds(tg, goal)
    # display path
    TileGame.print_pretty_path(path)



if __name__ == "__main__":
    main()
