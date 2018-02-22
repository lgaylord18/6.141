epsilon = 0.03; #radians, variable
target_radius = 0.02 #meters, variable
num_nodes = 5000;

def distance(p1, p2):
	return sqrt((p1[0]-ps[0])*(p1[0]-ps[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]));
	
def step_from_toward(p1, p2):
	if distance(p1, p2) < epsilon:
		return p2;
	else:
		theta = atan2(p2[1] - p1[1], p2[0] - p1[0]);
		return p1[0] + epsilon*cos(theta), p1[1] + epsilon*sin(theta);

class Node:
    def __init__(self, q, parent_id = None):
	self.q = q;
	self.parent_id = parent_id;
		
def find_nearest_node(nodes, rand):
	min_index = 0;
	for i, p in enumerate(nodes):
		if distance(q, p.q) < dist(rand, nodes[0].q):
			min_index = i;
		return min_index;
		
def backtrace(nodes, node):
	plan = [];
	curr_node = node;
	while True:
		plan.append(curr_node.q);
		if curr_node.parent_id is None:
			break;
		curr_node = nodes[curr_node.parent_id]
	
	plan.reverse();
	return plan;

def rrt(self):
	nodes = [Node(self.current_post)];
	
	for i in xrange(num_nodes):
		rand = #some random number
		nearest_node_index = find_nearest_node(nodes, rand);
		new_node = step_from_toward(nodes[nearest_node_index].q, rand);
		nodes.append(Node(new_node, nearest_node_index));
		
		if dist(new_node.q, self.target_pose) < target_radius:
			plan = backtrace(nodes, new_node);
			return plan;
		
	return None; #no plan found
