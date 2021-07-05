# 8Puzzle
Solved the 8 Puzzle Problem using A* Algorithm

Introduction :

Given a 3*3 frame of movable tiles with numbers from 1-8 and having one cell of the frame empty and making it possible to move the adjacent tiles in the empty. We can slide four adjacent (left, right, above and below) tiles into the empty space. The objective of the problem is to reach the arrangement where the tiles are arranged as per given the goal state.

A Star Algorithm :

A* search algorithm uses path-finding and graph traversal techniques to solve a given problem. It follows process of plotting efficiently traversable path between nodes. A* search algorithm keeps track on the nodes where it visits and therefore does not have to visit the node again which saves huge amount of time. Each node has pointer to the next node which is its parent to retrace the path to parent. The next node in the program is chosen from the f value where
f value = h value + g value h value -> heuristic value g value -> cost of the path

Program Structure :

Global variables:

initial_ar – start state
goal_ar – goal state

Local Variables:

state – current state
parent – parent node
move – shifting spaces either left,right,up or down
depth – depth of the node
step_cost – cost of the step taken
path_cost – actual cost of path g(n)
hcost – cost from current node to goal node also called h(n) blank_space – empty space
val_left – value of the left tile
new_state – new state of the tile
val_right – value of the left tile
val_down – value of the left tile
val_up – value of the left tile
goal_state – final state
heuristicFunct – heuristic function
cost – to remove 0's current position
curr – current state
i, j – loop variables
stateTrack – tracking of the state
moveTrack – tracking of the movement
depthTrack – tracking of the depth
count – counts the steps
start – counts the time
fringe – unvisited nodes
fringe_nodes_removed – number of nodes removed from the fringe fringe_maxLeng – maximum number of nodes
depth_fringe – fringe of node depth
path_cost_fringe – fringe of path cost
visited – visited states
curr_node – current node
curr_depth – current depth
curr_path_cost – current path cost
total_cost – total cost f(x)
path_cost – path cost g(x)

Functions :

check_right(self)
Used to check the empty space and move right.

check_left(self)
Used to check the empty space and move left.

check_up(self)
Used to check the empty space and move up.

check_down(self)
Used to check the empty space and move down.

calc_hcost(self, new_state, goal_state, heuristicFunct, path_cost, depth) 
Calculate cost of h(n) of specific input.

misplacedFunct(self, new_state, goal_state)
It returns heuristic function through misplaced tiles. 

manhattanFunct(self, new_state, goal_state)
It returns heuristic function through manhattan distance 

displayPath(self)
Used to find the path after reaching the goal.

a_starFunct(self, goal_state, heuristicFunct)
Performing A* search algorithm.
