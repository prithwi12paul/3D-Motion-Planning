import numpy as np

class RRTNode:
    def __init__(self, coord):
        self.coord = coord
        self.parent = None
        self.children = []

    
class RRT:

    @staticmethod
    def plan(start_coord, goal_coord, boundary, blocks, edge_length):
        goal_cost = None
        iter_max = 80000
        start_node = RRTNode(start_coord)
        goal_node = RRTNode(goal_coord)
        step_length = edge_length
        all_tree_nodes = [start_node]
        all_nodes_coord = set()
        for k in range(iter_max):
            sampled_point = RRT.generate_random_node(goal_coord,boundary,k)
            nearest_node = RRT.nearest_neighbor(sampled_point,all_tree_nodes)
            if RRT.check_collision(sampled_point, nearest_node, boundary, blocks,step_length) is False:
                new_node = RRT.get_new_node(nearest_node, sampled_point, step_length)
                new_node.parent = nearest_node
                if np.linalg.norm(new_node.coord - goal_coord) <=  step_length:
                    goal_node = new_node
                    break
            
                all_tree_nodes.append(new_node)
                all_nodes_coord.add(tuple(new_node.coord))
          
        goal_cost = RRT.total_cost(new_node)
        first_goal_cost = goal_cost
        first_path_to_goal = RRT.extract_path_to_goal(new_node)
        print("Total iterations: ", k)
        return first_goal_cost, first_path_to_goal

    @staticmethod
    def generate_random_node(goal_coord,boundary,iter):

      x_min,x_max = boundary[0],boundary[3]
      y_min,y_max = boundary[1],boundary[4]
      z_min,z_max = boundary[2],boundary[5]

      if iter % 10 == 0:

        sampled_x = goal_coord[0]
        sampled_y = goal_coord[1]
        sampled_z = goal_coord[2]

      else:

        sampled_x = np.random.uniform(x_min,x_max)
        sampled_y = np.random.uniform(y_min,y_max)
        sampled_z = np.random.uniform(z_min,z_max)

      return (sampled_x,sampled_y,sampled_z)


    @staticmethod
    def nearest_neighbor(point,all_tree_nodes):
        return all_tree_nodes[np.argmin([np.linalg.norm(nd.coord-np.array(point)) for nd in all_tree_nodes])]

    @staticmethod
    def check_collision(sampled_point, nearest_node, boundary, blocks, step_length):
      sample_x = sampled_point[0]
      sample_y = sampled_point[1]
      sample_z = sampled_point[2]

      if (sample_x > boundary[0] and sample_x < boundary[3] and sample_y > boundary[1] and sample_y < boundary[4] and sample_z > boundary[0] 
 and sample_z < boundary[5]):
          for i in range(len(blocks)):
            obstacle = blocks[i]
            if obstacle[0] <= sample_x <= obstacle[3] and obstacle[1] <= sample_y <= obstacle[4] and obstacle[2] <= sample_z <= obstacle[5]:
                return True
            step = 1
            steps = 10
            while  step <= steps :
                next_point = nearest_node.coord  + (step_length/steps) * step * RRT.unit_vector(nearest_node.coord,sampled_point)
                if obstacle[0] <= next_point[0] <= obstacle[3] and obstacle[1] <= next_point[1] <= obstacle[4] and obstacle[2] <= next_point[2] <= obstacle[5]:
                    return True
                step = step + 1
          return False
      else:
        return True
    
    @staticmethod
    def get_new_node(nearest_node, sampled_point,step_length):
        new_node_coord = nearest_node.coord + step_length * RRT.unit_vector(nearest_node.coord,sampled_point)
        new_node= RRTNode(new_node_coord)
        return new_node

    @staticmethod
    def unit_vector(node_start, node_end):
        direction = (np.array(node_end) - np.array(node_start))/ np.linalg.norm(np.array(node_end) - np.array(node_start))
        return direction

    @staticmethod
    def total_cost(node):
        cost = 0
        while node.parent:
            cost += np.linalg.norm(node.coord-node.parent.coord)
            node = node.parent
        return cost    
    
    @staticmethod
    def extract_path_to_goal(new_node):
        path = []
        node = new_node
        while node.parent is not None:
            path.append(node.coord)
            node = node.parent
        path.append(node.coord)
        return path
