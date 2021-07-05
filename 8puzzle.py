import numpy as np
import time

class main_astar():
    def __init__(self, state, parent, move, depth, step_cost, path_cost, hcost):
        self.state = state
        self.parent = parent  #parent node
        self.move = move  # shifting spaces either left,right,up or down
        self.depth = depth  # depth of the node 
        self.step_cost = step_cost  # g(n)
        self.path_cost = path_cost  # Actual cost g(n)
        self.hcost = hcost  # cost from current node to goal node also called h(n)

        #Adjacent child nodes
        self.shift_up = None
        self.shift_left = None
        self.shift_down = None
        self.shift_right = None

    # move down
    def check_down(self):
        # blank space = 0
        blank_space = [i[0] for i in np.where(self.state == 0)]
        if blank_space[0] == 0:
            return False
        else:
            val_up = self.state[blank_space[0] - 1, blank_space[1]]  # value above
            new_state = self.state.copy()
            new_state[blank_space[0], blank_space[1]] = val_up
            new_state[blank_space[0] - 1, blank_space[1]] = 0
            return new_state, val_up

    # move right
    def check_right(self):
        blank_space = [i[0] for i in np.where(self.state == 0)]
        if blank_space[1] == 0:
            return False
        else:
            val_left = self.state[blank_space[0], blank_space[1] - 1]  # value of the left tile
            new_state = self.state.copy()
            new_state[blank_space[0], blank_space[1]] = val_left
            new_state[blank_space[0], blank_space[1] - 1] = 0
            return new_state, val_left

    # move up
    def check_up(self):
        blank_space = [i[0] for i in np.where(self.state == 0)]
        if blank_space[0] == 2:
            return False
        else:
            val_down = self.state[blank_space[0] + 1, blank_space[1]]  # value of the lower tile
            new_state = self.state.copy()
            new_state[blank_space[0], blank_space[1]] = val_down
            new_state[blank_space[0] + 1, blank_space[1]] = 0
            return new_state, val_down

    # move left
    def check_left(self):
        blank_space = [i[0] for i in np.where(self.state == 0)]
        if blank_space[1] == 2:
            return False
        else:
            val_right = self.state[blank_space[0], blank_space[1] + 1]  # value of the right tile
            new_state = self.state.copy()
            new_state[blank_space[0], blank_space[1]] = val_right
            new_state[blank_space[0], blank_space[1] + 1] = 0
            return new_state, val_right

    #returns h(n) of specific input
    def calc_hcost(self, new_state, goal_state, heuristicFunct, path_cost, depth):
        if heuristicFunct == 'misplacedSpaces':
            return self.misplacedFunct(new_state, goal_state)
        elif heuristicFunct == 'manhattan':
            return self.manhattanFunct(new_state, goal_state) - path_cost + depth

    # Heuristic function: Method 1 : Misplaced Tiles
    def misplacedFunct(self, new_state, goal_state):
        cost = np.sum(new_state != goal_state) - 1  # To remove 0's current position
        if cost > 0:
            return cost
        else:
            return 0  # when all current state = goal state

    # Heuristic function: Method 2 : Manhattan Distance
    def manhattanFunct(self, new_state, goal_state):
        curr = new_state
        #Original positions
        goalPOS = {1: (0, 0), 2: (0, 1), 3: (0, 2), 8: (1, 0), 0: (1, 1), 4: (1, 2), 7: (2, 0), 6: (2, 1),
                             5: (2, 2)}
        total = 0
        for i in range(3):
            for j in range(3):
                if curr[i, j] != 0:
                    total += sum(abs(a - b) for a, b in zip((i, j), goalPOS[curr[i, j]]))
        return total

    # To find the path after reaching the goal
    def displayPath(self):
        print ("\nGoal state reached!")
        stateTrack = [self.state]
        moveTrack = [self.move]
        depthTrack = [self.depth]
        # Add info while going back
        while self.parent:
            self = self.parent
            stateTrack.append(self.state)
            moveTrack.append(self.move)
            depthTrack.append(self.depth)
        # print the path
        count = 0
        while stateTrack:
            print ('step', count)
            print (stateTrack.pop())
            print ('move=', moveTrack.pop(), ', depth=', str(depthTrack.pop()), '\n')
            count += 1

    #A star search
    def a_starFunct(self, goal_state, heuristicFunct):
        start = time.time()

        fringe = [
            (self, 0)]  #Unvisited nodes, fringe = fringeueue
        fringe_nodes_removed = 0  # number of nodes removed from the fringeueue
        fringe_maxLeng = 1  # Max no of nodes

        depth_fringe = [(0, 0)]  # fringe of node depth, (depth, f(n))
        path_cost_fringe = [(0, 0)]  # fringe for path cost, (path_cost, f(n))
        visited = set([])  # visited states

        while fringe:
            # sort fringe in increasing order
            fringe = sorted(fringe, key=lambda x: x[1])
            depth_fringe = sorted(depth_fringe, key=lambda x: x[1])
            path_cost_fringe = sorted(path_cost_fringe, key=lambda x: x[1])

            # Maximum length
            if len(fringe) > fringe_maxLeng:
                fringe_maxLeng = len(fringe)

            curr_node = fringe.pop(0)[0]  #pop the first node from the queue
            fringe_nodes_removed += 1
            curr_depth = depth_fringe.pop(0)[0]  #pop the depth for curr node
            curr_path_cost = path_cost_fringe.pop(0)[0]  #pop the path cost for reaching curr node
            visited.add(
                tuple(curr_node.state.reshape(1, 9)[0]))  # To avoid revisiting the node visited already states
            #print visited
            # If goal state is found, move back to the root node and display path
            if np.array_equal(curr_node.state, goal_state):
                curr_node.displayPath()
                print ('Nodes Generated', str(fringe_nodes_removed))
                print ('Nodes Expanded:', str(fringe_maxLeng))
                print ('Time: %0.2fs' % (time.time() - start))
                return True

            else:
                # check if upper tile to down is ok
                if curr_node.check_down():
                    new_state, val_up = curr_node.check_down()
                    # check if node visited already
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = curr_path_cost + val_up
                        depth = curr_depth + 1
                        # get heuristic cost
                        h_cost = self.calc_hcost(new_state, goal_state, heuristicFunct, path_cost, depth)
                        total_cost = path_cost + h_cost
                        curr_node.shift_down = main_astar(state=new_state, parent=curr_node, move='down', depth=depth,step_cost=1, path_cost=path_cost, hcost=h_cost)
                        fringe.append((curr_node.shift_down, total_cost))
                        depth_fringe.append((depth, total_cost))
                        path_cost_fringe.append((path_cost, total_cost))

                # check if left tile to right is ok
                if curr_node.check_right():
                    new_state, val_left = curr_node.check_right()
                    # check if  node visited already
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = curr_path_cost + val_left
                        depth = curr_depth + 1
                        # get heuristic cost
                        h_cost = self.calc_hcost(new_state, goal_state, heuristicFunct, path_cost, depth)
                        total_cost = path_cost + h_cost
                        curr_node.shift_right = main_astar(state=new_state, parent=curr_node, move='right',
                                                       depth=depth, \
                                                       step_cost=1, path_cost=path_cost, hcost=h_cost)
                        fringe.append((curr_node.shift_right, total_cost))
                        depth_fringe.append((depth, total_cost))
                        path_cost_fringe.append((path_cost, total_cost))

                # check if bottom tile to up is ok
                if curr_node.check_up():
                    new_state, val_down = curr_node.check_up()
                    # check if  node visited already
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = curr_path_cost + val_down
                        depth = curr_depth + 1
                        # get heuristic cost
                        h_cost = self.calc_hcost(new_state, goal_state, heuristicFunct, path_cost, depth)
                        total_cost = path_cost + h_cost
                        curr_node.shift_up = main_astar(state=new_state, parent=curr_node, move='up', depth=depth,step_cost=1, path_cost=path_cost, hcost=h_cost)
                        fringe.append((curr_node.shift_up, total_cost))
                        depth_fringe.append((depth, total_cost))
                        path_cost_fringe.append((path_cost, total_cost))

                # check if right tile to left is ok
                if curr_node.check_left():
                    new_state, val_right = curr_node.check_left()
                    # check if  node visited already
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = curr_path_cost + val_right
                        depth = curr_depth + 1
                        # get heuristic cost
                        h_cost = self.calc_hcost(new_state, goal_state, heuristicFunct, path_cost, depth)
                        total_cost = path_cost + h_cost
                        curr_node.shift_left = main_astar(state=new_state, parent=curr_node, move='left', depth=depth, step_cost=1, path_cost=path_cost, hcost=h_cost)
                        fringe.append((curr_node.shift_left, total_cost))
                        depth_fringe.append((depth, total_cost))
                        path_cost_fringe.append((path_cost, total_cost))


print ("Enter the start state")
initial_ar = []
for i in range(0,9):
	ele = int(input())
	initial_ar.append(ele)
initial_state=np.array([initial_ar]).reshape(3,3)
print (initial_state)

print ("Enter the goal state")
goal_ar = []
for i in range(0,9):
	ele1 = int(input())
	goal_ar.append(ele1)
goal_state=np.array([goal_ar]).reshape(3,3)
print (goal_state)
print ('Wait. Calculating........................\n')
root = main_astar(state=initial_state,parent=None,move=None,depth=0,step_cost=0, path_cost=0,hcost=0)
ans = int(input('\nEnter 1.Misplaced Tiles\n2.Manhattan Distance:\n'))
if ans==1:
	root.a_starFunct(goal_state,heuristicFunct = 'misplacedSpaces')
elif ans==2:
	root.a_starFunct(goal_state,heuristicFunct = 'manhattan')