import numpy as np
import matplotlib.pyplot as plt
import os

class AStar(object):
    def __init__(self, field, start, goal, cost_map = None):
        self.field = field
        if cost_map is not None and cost_map.shape != field.shape:
            raise ValueError("cost_map and field should be same shape.")
        self.cost_map = cost_map
        self.node_field = self.__make_node_field(field)
        
        self.open_list = []
        self.close_list = []
        
        self.start = start
        self.goal = goal
        
        #open_listにstartを追加
        self.open_list.append(self.start)
        self.node_field[self.start[0], self.start[1]].set_path(0)
        
        #for plot
        self.plot_field = np.zeros_like(self.field)
        self.plot_field[self.field == 1] = 0.3
    
    def __make_node_field(self, field):
        node_field = np.array([[Node(coord=np.array([i, j])) for j in range(field.shape[1])] for i in range(field.shape[1])], dtype=Node)
        node_field[field == 0] = None
        return node_field
    
    def __search_init(self, coord):
        self.node_field[coord[0], coord[1]].set_status("open")
        self.node_field[coord[0], coord[1]].set_path(0)
        self.node_field[coord[0], coord[1]].set_cost(self.cost_func())
    
    def set_start(self, start):
        self.start = start
    
    def set_goal(self, goal):
        self.goal = goal
    
    def set_cost_function(self, func_name):
        if func_name in ["cartesian", "Cartesian", "c", "C"]:
            self.cost_func_name = "Cartesian"
            self.cost_func = self.__calc_cartesian
        elif func_name in ["manhattan", "Manhattan", "m", "M"]:
            self.cost_func_name = "Manhattan"
            self.cost_func = self.__calc_manhattan
        else :
                raise ValueError("func_name should be cartesian or manhattan")
            
    def __calc_cartesian(self, coord):
        return np.sqrt(np.sum((coord - self.goal)**2))
    
    def __calc_manhattan(self, coord):
        return np.sum(np.abs(coord - self.goal))
    
    def search(self, save=False):
        self.node_field[self.start[0], self.start[1]].set_cost(self.cost_func(self.start))
        count = 0
        
        if save:
            if not os.path.exists("./astar_graph"):
                os.mkdir("./astar_graph")
        while len(self.open_list) != 0:
            ref_node_coord = self.get_new_ref_node()
            goal_flag = self.open_nodes(ref_node_coord)
            #周りを探索したnodeをcloseに
            self.node_field[ref_node_coord[0], ref_node_coord[1]].set_status("close")
            self.close_list.append(ref_node_coord)
            
            #for plot
            self.plot_field[ref_node_coord[0], ref_node_coord[1]] = 1
            self.plot_field[self.start[0], self.start[1]] = 0.5
            self.plot_field[self.goal[0], self.goal[1]] = 0.8
            if save:
                plt.pcolor(self.plot_field)
                plt.savefig("./astar_graph/astar_graph_{}.png".format(10000 + count))
                plt.close()
                count = count + 1
            if goal_flag:
                break
        else:
            raise SystemExit("Goal is not found.")
        
        self.plot_search_area()
        self.plot_solution(save, count)
       
    def plot_search_area(self):
        plt.pcolor(self.plot_field)
        plt.show()
        plt.close()
    
    def plot_solution(self, save, count):
        self.plot_field = np.zeros_like(self.field)
        self.plot_field[self.field == 1] = 0.3
        self.plot_field[self.start[0], self.start[1]] = 0.5
        self.plot_field[self.goal[0], self.goal[1]] = 0.8
        
        current_point = self.goal
        while (current_point != self.start).any():
            parent_node = self.node_field[current_point[0], current_point[1]].get_parent_node()
            current_point = parent_node.get_coord()
            self.plot_field[current_point[0], current_point[1]] = 1
        
        plt.pcolor(self.plot_field)
        if save :
            plt.savefig("./astar_graph/astar_graph_{}.png".format(10000 + count))
        
    
    def  get_new_ref_node(self):
        self.open_list.sort(key = lambda node:self.node_field[node[0], node[1]].get_cost() + self.node_field[node[0], node[1]].get_path())
        new_ref_node_coord = self.open_list[0]
        del self.open_list[0]
        return new_ref_node_coord
    
    def open_nodes(self, ref_coord):
        #goal判定
        goal_flag = False
        ref_node = self.node_field[ref_coord[0], ref_coord[1]]
        check_node_list = list(self.node_field[[ref_coord[0]-1, ref_coord[0]+1], ref_coord[1]].reshape(-1, ))
        check_node_list += (list(self.node_field[ref_coord[0], [ref_coord[1]-1, ref_coord[1]+1]].reshape(-1, )))
        for node in check_node_list:
            if node is None:
                continue
            if node.get_status() == None:
                node.set_cost(self.cost_func(node.get_coord()))
                node.set_path(ref_node.path + 1)
                node.set_status("open")
                node.set_parent_node(ref_node)
                self.open_list.append(node.get_coord())
                
                if (node.get_coord() ==  self.goal).all():
                    goal_flag = True
   
        return goal_flag


class Node(object):
    def __init__(self, coord):
        self.status = None
        self.coord = coord
        self.path = None
        self.cost = None
        self.parent_node = None
        
    def __str__(self):
        return "ASter Node : coord ({}, {})".format(self.coord[0], self.coord[1])
    
    def __repr__(self):
        return "ASter Node : coord ({}, {})".format(self.coord[0], self.coord[1])
    
    def set_status(self, status):
        self.status = status
    
    def get_status(self):
        return self.status
    
    def set_path(self, path):
        self.path = path
        
    def get_path(self):
        return self.path
    
    def set_cost(self, cost):
        self.cost = cost
    
    def get_cost(self):
        return self.cost
    
    def get_coord(self):
        return self.coord
    
    def set_parent_node(self, node):
        self.parent_node = node
        
    def get_parent_node(self):
        return self.parent_node