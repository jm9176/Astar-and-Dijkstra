from waypoint import Waypoint
import math
import numpy as np

class PathFinder(object):

    def get_path(self, grid, start_wp, end_wp):

        # Defining the start position
        x_init = start_wp.x
        y_init = start_wp.y

        #Defining the goal position
        x_goal = end_wp.x
        y_goal = end_wp.y

        # List containg all the nodes available to select
        # equal to the open_list but without the locations
        # outside the bounds
        open_list = [Waypoint(x_init, y_init)]
        closed_list = []

        # Determining the grid size
        shape=np.shape(grid)
        rows = shape[0]
        cols = shape[1]
              
        parent = [[]]

        # calculating the neighbors
        def neighbor(curr_var):

            x = curr_var.x
            y = curr_var.y
            
            edge_list = [Waypoint(x-1,y+1), Waypoint(x,y+1),
                         Waypoint(x-1,y), Waypoint(x+1, y),
                         Waypoint(x+1,y+1), Waypoint(x-1,y-1),
                         Waypoint(x,y-1), Waypoint(x+1,y-1)]
     
            return edge_list

        # removing all the points which are out of bounds or are obstacles
        def out_of_bound(list_x, grid, rows, cols):
            edge_list = []
            
            for var in list_x:
                if var.x >0 and var.y>0:
                    if var.x<rows and var.y < cols and grid[var.x][var.y]==False:
                         edge_list.append(var)
                         
            return edge_list

        # Calculating the heuristic values
        def heuristic(var1, var2):
            return math.sqrt((math.pow(var1.x-var2.x, 2)) + (math.pow(var1.y-var2.y,2)))

        # Function for the backtracking path
        def final_path(cameFrom, current):
            total_path = [current]
            while current in cameFrom.Keys:
                total_path.append(current)
            
            return total_path
       
        # Defining the edge cost other that the start node to Inf
        # This value represent the Gscore of the cost function
        gScore = np.full((rows,cols),np.inf)
        fScore = np.full((rows,cols),np.inf)
        
        gScore[x_init][y_init] = 0
        fScore[x_init][y_init] = heuristic(start_wp, end_wp)

        # variable to increment the index of the parent list 
        count = 0

        # flag to check whether the variable is found or not
        # while back tracking
        flag = 0
        
        while open_list is not None:
            # Selecting the node from the open list with low
            # fScore value
            
            temp_val= fScore[open_list[0].x][open_list[0].y]
            curr_var = open_list[0]
            
            for var in open_list:
                if fScore[var.x][var.y] < temp_val:
                    temp_val = fScore[var.x][var.y]
                    curr_var = var
                  
            if curr_var==end_wp:
                path = []
                current = curr_var
                temp_val = gScore[current.x][current.y]

                for l in range(len(parent)):
                    path.append(current)
                    
                    for i in range(len(parent)):
                        for j in range(1,len(parent[i])):
                            if parent[i][j] == current and gScore[parent[i][0].x][parent[i][0].y] <= temp_val:
                                temp_var = parent[i][0]
                                temp_val = gScore[parent[i][0].x][parent[i][0].y]
                                
                    current = temp_var
                                   
                    if current == start_wp:
                        path.append(start_wp)
                        break

                return path[::-1]
            
            open_list.remove(curr_var)
            closed_list.append(curr_var)

            edge_list = neighbor(curr_var)
            edge_list = out_of_bound(edge_list, grid, rows, cols)
            parent[count].append(curr_var)
            parent[count].extend(edge_list)
   
            count += 1
            
            # Updating the open_list and the fScore value
            for var in edge_list:
                if var in closed_list:
                    continue

                temp_gScore = gScore[curr_var.x][curr_var.y] + heuristic(curr_var, var)

                if var not in open_list:
                    open_list.append(var)
                    
                if temp_gScore <= gScore[var.x][var.y]:
                    gScore[var.x][var.y] = temp_gScore

                #fScore calculation for A*
                #fScore[var.x][var.y] = gScore[var.x][var.y] + heuristic(var, end_wp)

                #fScore calculation for Dijkstra
                fScore[var.x][var.y] = gScore[var.x][var.y]
                
            if curr_var == end_wp:
                break
            
            parent.append([])
