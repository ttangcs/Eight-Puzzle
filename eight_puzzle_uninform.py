import numpy as np
from heapq import heappush, heappop
from animation import draw
import argparse

#Thomas Tang

class Node():
    """
    cost_from_start - the cost of reaching this node from the starting node
    state - the state (row,col)
    parent - the parent node of this node, default as None
    """
    def __init__(self, state, cost_from_start, parent = None):
        self.state = state
        self.parent = parent
        self.cost_from_start = cost_from_start


class EightPuzzle():
    
    def __init__(self, start_state, goal_state, algorithm, array_index):
        self.start_state = start_state
        self.goal_state = goal_state
        self.visited = [] 
        self.algorithm = algorithm
        self.array_index = array_index

    # goal test function
    def goal_test(self, current_state):
        return np.array_equal(current_state, self.goal_state)



    # get cost function
    def get_cost(self, current_state, next_state):
        if np.array_equal(current_state, next_state):
            return 0
        return 1


    # get successor function
    def get_successors(self, state):
        successors = []
        # your code goes here:
        rows, columns = np.where(state == 0)

        if rows[0] + 1 < 3:
            copyOfState = np.copy(state)
            value = state[rows[0]][columns[0]]
            replacementValue = state[rows[0] + 1][columns[0]]
            copyOfState[rows[0]][columns[0]] = replacementValue
            copyOfState[rows[0] + 1][columns[0]] = value
            successors.append(copyOfState)

        if rows[0] - 1 >= 0:
            copyOfState = np.copy(state)
            value = state[rows[0]][columns[0]]
            replacementValue = state[rows[0] - 1][columns[0]]
            copyOfState[rows[0]][columns[0]] = replacementValue
            copyOfState[rows[0] - 1][columns[0]] = value
            successors.append(copyOfState)

        if columns[0] + 1 < 3:
            copyOfState = np.copy(state)
            value = state[rows[0]][columns[0]]
            replacementValue = state[rows[0]][columns[0] + 1]
            copyOfState[rows[0]][columns[0]] = replacementValue
            copyOfState[rows[0]][columns[0] + 1] = value
            successors.append(copyOfState)

        if columns[0] - 1 >= 0:
            copyOfState = np.copy(state)
            value = state[rows[0]][columns[0]]
            replacementValue = state[rows[0]][columns[0] - 1]
            copyOfState[rows[0]][columns[0]] = replacementValue
            copyOfState[rows[0]][columns[0] - 1] = value
            successors.append(copyOfState)
        
        return successors

    # get priority of node for UCS
    def priority(self, node):
        count = 0

        copyOfNode = np.copy(node)
        copyOfGoal = np.copy(self.goal_state)
        
        
        
        for x in range(3):
            for y in range(3):
                if copyOfNode[x][y] != copyOfGoal[x][y]:
                    count += 1
        return  count


    
    # draw 
    # you do not need to modify anything in this function.
    def draw(self, node):
        path=[]
        while node.parent:
            path.append(node.state)
            node = node.parent
        path.append(self.start_state)
        draw(path[::-1], self.array_index, self.algorithm)

    # solve it
    def solve(self):
        container = [] # node
        count = 1
        state = self.start_state.copy()
        node = Node(state, 0, None)
        self.visited.append(state)
        if self.algorithm == 'Depth-Limited-DFS': 
            maximumDepth = 15
            container.append(node)


        elif self.algorithm == 'BFS': 
            container.insert(0, node)
            

        elif self.algorithm == 'UCS': 
            heappush(container,(100000, count, node))
            
        
        while container: 
            # your code goes here:
            # if one solution is found, call self.draw(current_node) to show and save the animation. 
            
            if self.algorithm == 'Depth-Limited-DFS':
                currentNode = container.pop()
                self.visited.append(currentNode.state)
                successors = self.get_successors(currentNode.state)
                newCost = currentNode.cost_from_start + 1
                 
                if newCost <= maximumDepth:
                    pass
                else:
                    continue


            elif self.algorithm == 'BFS':
                currentNode = container.pop()
                self.visited.append(currentNode.state)
                successors = self.get_successors(currentNode.state)
                newCost = currentNode.cost_from_start + 1

            elif self.algorithm == 'UCS':
                print("Loop")
                newCost, count, currentNode = heappop(container)
                self.visited.append(currentNode.state)
                successors = self.get_successors(currentNode.state)
                newCost = currentNode.cost_from_start + 1
                count += 1


            for successor in successors:
                if self.goal_test(successor):
                    self.draw(Node(successor, newCost, currentNode))
                    return

                isVisited = False
                for visited in self.visited:
                    if np.array_equal(visited, successor):
                        isVisited = True

                if self.algorithm == 'Depth-Limited-DFS':
                    if not isVisited:
                        container.append(Node(successor,newCost, currentNode))
                elif self.algorithm == 'BFS':
                    if not isVisited:
                        container.insert(0,Node(successor, newCost, currentNode))
                elif self.algorithm == 'UCS':
                    if not isVisited and (self.priority(successor) < self.priority(currentNode.state)):
                        heappush(container, (self.priority(successor), count, Node(successor, newCost, currentNode)))



            
            
# You do not need to change anything below this line, except for debuggin reason.
if __name__ == "__main__":
    
    goal = np.array([[1,2,3],[4,5,6],[7,8,0]])

    start_arrays = [np.array([[0,1,3],[4,2,5],[7,8,6]]), 
                    np.array([[0,2,3],[1,4,6],[7,5,8]])] 

    algorithms = ['Depth-Limited-DFS', 'BFS', 'UCS']

    parser = argparse.ArgumentParser(description='eight puzzle')

    parser.add_argument('-array', dest='array_index', required = True, type = int, help='index of array')
    parser.add_argument('-algorithm', dest='algorithm_index', required = True, type = int, help='index of algorithm')

    args = parser.parse_args()

    # run this in the terminal using array 0, algorithm BFS
    # python eight_puzzle_uninform.py -array 0 -algorithm 1
    game = EightPuzzle(start_arrays[args.array_index], goal, algorithms[args.algorithm_index], args.array_index)
    game.solve()