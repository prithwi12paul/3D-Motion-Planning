
# priority queue for OPEN list
from pqdict import pqdict
import math
import numpy as np
import matplotlib.pyplot as plt
# import matplotlib.pyplot as plt

class AStarNode(object):
  def __init__(self,coord, hval):
    self.coord = coord
    self.g = np.inf
    self.h = hval
    self.parent_node = set()
    self.parent_action = None
    self.closed = False

  def __lt__(self, other):
    return self.g < other.g     


class AStar(object):

  @staticmethod
  def plan(start_coord, boundary, blocks, goal_coord, offset,epsilon = 1 ):
    Graph = {}
    OPEN = pqdict()
    CLOSED = set()

    
    # TODO: Implement A* here

    # Initializing the OPEN list with the start node

    start_node = AStarNode(tuple(start_coord), AStar.get_heuristic(start_coord,goal_coord))
    goal_node = AStarNode(tuple(goal_coord), AStar.get_heuristic(goal_coord,goal_coord))
    start_node.g = 0

    OPEN[start_node] = start_node.g + start_node.h  ## storing the f value
    iter = 0

    while tuple(goal_coord) not in CLOSED and len(OPEN) !=0:
      iter = iter + 1
      curr_cell = OPEN.popitem()
      curr_cell_node = curr_cell[0]
      curr_cell_node_coord = curr_cell_node.coord
      curr_cell_fval = curr_cell[1]
      CLOSED.add(tuple(curr_cell_node_coord))

      if np.linalg.norm(curr_cell_node_coord - goal_coord) <= 0.5 * offset:
        # goal_node = curr_cell_node
        break
    
  

      
      neighbors = AStar.children_coord(curr_cell_node_coord,offset)

      for child_coord in neighbors:
        
        if AStar.check_collision(child_coord,curr_cell_node,boundary,blocks, offset) is False:
    

          if child_coord not in CLOSED:
          
              count = 0
              for key in OPEN:
                if tuple(key.coord) == tuple(child_coord):
                  child_node = key
                  count = 1
                  break
              if count == 0:
                child_node = AStarNode(tuple(child_coord), AStar.get_heuristic(child_coord,goal_coord))

              if child_node.g > curr_cell_node.g + AStar.stage_cost(curr_cell_node_coord, child_coord):
                child_node.g = curr_cell_node.g + AStar.stage_cost(curr_cell_node_coord, child_coord)
                child_node.parent_node.add(curr_cell_node)
                OPEN[child_node] = child_node.g + epsilon * child_node.h  ## updating the F value of the child node
        
    
    
    node = curr_cell_node
    path = [curr_cell_node.coord]
    print("Number of nodes in the CLOSED set", len(CLOSED))

    while len(node.parent_node)!=0:

      g_val = [ n.g  for n in node.parent_node]
      best_node_idx = np.argmin(g_val)
      best_node = list(node.parent_node)[best_node_idx]
      path.append(best_node.coord)
      node = best_node
    
    print(path[::-1])

    return path[::-1]

  
  @staticmethod
  def children_coord(coord,offset):

    neighbors = []
    x,y,z = coord
    offsets = [-offset, 0, offset]
    for dx in offsets:
      for dy in offsets:
        for dz in offsets:
          if dx == 0  and dy == 0 and dz == 0:
            continue
          neighbors.append(tuple(np.round((x+dx,y+dy,z+dz),decimals = 1)))

    return neighbors
  
  @staticmethod
  def stage_cost(p1,p2):
    return np.linalg.norm(np.array(p1)-np.array(p2))
  
  @staticmethod
  def get_heuristic(p1,p2):
    return np.linalg.norm(np.array(p1)-np.array(p2))
  
  @staticmethod
  def unit_vector(node_start, node_end):
      direction = -(np.array(node_end) - np.array(node_start))/ np.linalg.norm(np.array(node_end) - np.array(node_start))
      return direction

  @staticmethod
  def check_collision(sampled_point, prev_node, boundary, blocks, step_length):
    sample_x = sampled_point[0]
    sample_y = sampled_point[1]
    sample_z = sampled_point[2]

    temp = prev_node.coord
    next_point = temp

    if (sample_x > boundary[0] and sample_x < boundary[3] and sample_y > boundary[1] and sample_y < boundary[4] and sample_z > boundary[2] 
and sample_z < boundary[5]):
        for i in range(len(blocks)):
          obstacle = blocks[i]
          if obstacle[0] <= sample_x <= obstacle[3] and obstacle[1] <= sample_y <= obstacle[4] and obstacle[2] <= sample_z <= obstacle[5]:
              return True
          
          # # else:
          step = 1
          steps = 10
          while  step <= steps :
            next_point = prev_node.coord  + (step_length/steps) * step * AStar.unit_vector(prev_node.coord,sampled_point)
            if obstacle[0] <= next_point[0] <= obstacle[3] and obstacle[1] <= next_point[1] <= obstacle[4] and obstacle[2] <= next_point[2] <= obstacle[5]:
                return True
            step = step + 1
        return False
    else:
      return True



if __name__=='__main__':

  start = np.array([0, 0, 0])
  goal = np.array([3,0,0])

  path= AStar.plan(start_coord = start, goal_coord = goal, offset = 0.1, epsilon = 1)
  print(path[::-1])
  path = np.array(path[::-1])
  fig = plt.figure()
  ax = plt.axes(projection='3d')

  ax.plot3D(path[:,0],path[:,1],path[:,2])
  plt.show()
