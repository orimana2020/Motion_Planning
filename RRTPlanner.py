import numpy
from RRTTree import RRTTree

from matplotlib import pyplot as plt

class RRTPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.p_bias = 0.05#0.05
        self.k = -1
        
    def Plan(self, start_config, goal_config, step_size =  0.001):
        
        step_size = None #TODO: Delete

        goal_config = [goal_config[1], goal_config[0]]
        start_config = [start_config[1], start_config[0]]
        # Initialize an empty plan.
        plan = []

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)

        n = 100 #100000
        is_goal_connected = False
        #Building tree:
        while True:
        #for i in range(n):
            #should be one if there was a success - the goal was chosen.
            x_random = numpy.random.binomial(1, self.p_bias)
            if (x_random == 1):
                x_random = goal_config
            else:
                while x_random == 0 or (not self.planning_env.state_validity_checker(x_random)):
                    random_sample_x =  numpy.random.uniform(self.planning_env.xlimit[0], 
                        self.planning_env.xlimit[1] + 1)
                    random_sample_y =  numpy.random.uniform(self.planning_env.ylimit[0], 
                        self.planning_env.ylimit[1] + 1)
                    x_random = (random_sample_x, random_sample_y)
            id_x_near, x_near = self.tree.GetNearestVertex(x_random)
            self.x_near = numpy.array(x_near)
            self.x_random = numpy.array(x_random)
            self.step_size = step_size
            if (step_size != None):
                x_new = self.extend()
            else:
                x_new = numpy.array(x_random)
            if self.planning_env.edge_validity_checker(x_near, x_new):

                id_x_new = self.tree.AddVertex(list(x_new))
                self.tree.AddEdge(id_x_near, id_x_new)
                if ((x_new == goal_config).all()):
                    goal_id = id_x_new
                    is_goal_connected = True
                    break

                if(self.k > -1):
                    k = min(self.k, len(self.tree.vertices)-1)
                    x_near_neighbors,_ = self.tree.GetKNN(x_new, k)
                    for neighbor in x_near_neighbors:
                        self.rewire_rrt_star(neighbor, id_x_new)
                        self.rewire_rrt_star(id_x_new, neighbor)

        if not is_goal_connected:
            return []

        # TODO (student): Implement your planner here.
        plan, _ = self.get_shortest_path(goal_id)

        for child, parent in self.tree.edges.items():
            plot_x = [self.tree.vertices[parent][1], self.tree.vertices[child][1]]
            plot_y = [self.tree.vertices[parent][0], self.tree.vertices[child][0]]
            plt.plot(plot_y, plot_x,'-or',markersize=1)
        # plan.append(start_config)

        # plan.append(goal_config)
        return numpy.array(plan)


    def rewire_rrt_star (self, x_potential_parent_id, x_child_id):
        x_potential_parent = self.tree.vertices[x_potential_parent_id]
        x_child = self.tree.vertices[x_child_id]
        if self.planning_env.edge_validity_checker(x_potential_parent, x_child):
            c = self.planning_env.compute_distance(x_potential_parent, x_child)
            _, cost_parent = self.get_shortest_path(x_potential_parent_id)
            _, cost_child = self.get_shortest_path(x_child_id)
            if (cost_parent + c < cost_child):
                self.tree.AddEdge(x_potential_parent_id, x_child_id)


    def line_cost (self, line: tuple()):
        return self.planning_env.compute_distance(self.tree.vertices[line[0]],
            self.tree.vertices[line[1]])

    def add_line_to_graph (self, key, line, graph):
        if key not in graph.keys():
            graph[key] = []
        graph[key].append(line)

    #In a tree there is only one path for a node from root
    def get_shortest_path(self, dest):
        plan = []
        cost = 0
        while (dest != self.tree.GetRootID()):
            parent_id = self.tree.edges[dest]
            parent_vertex = self.tree.vertices[parent_id].copy()
            #parent_vertex.reverse()
            child_vertex = self.tree.vertices[dest].copy()
            plan.append(child_vertex)
            #child_vertex.reverse()
            cost += self.planning_env.compute_distance(parent_vertex, child_vertex)
            dest = parent_id
        #plan = []
        plan.append(self.tree.vertices[dest])
        return plan, cost
        
        # shortest_path = []
        # #A dict where for each point that we didn't visit yet, saves the lowest cost it found until now.
        # unvisited_points = {}

        # #building graph
        # graph = {}
        # for line in self.tree.edges.items():
        #     first_point = tuple(self.tree.vertices[line[0]])
        #     second_point = tuple(self.tree.vertices[line[1]])
        #     self.add_line_to_graph(first_point, (line, second_point), graph)
        #     self.add_line_to_graph(second_point, (line, first_point), graph)
        #     unvisited_points[first_point] = 'inf'
        #     unvisited_points[second_point] = 'inf'

        # source = tuple(source)
        # dest = tuple(dest)
        # current_point = source
        # current_cost = 0
        # #A dict where each point is connected to the father that gives the lowest cost.
        # shortest_path_points = {}
        # if (len(unvisited_points) == 0 or source == None or dest == None):
        #     return [], 0

        # while(len(unvisited_points) != 0):
        #     if current_cost == 'inf':
        #         return [], 0
        #     if current_point == dest:
        #         break
        #     unvisited_points.pop(current_point)
        #     for line, neighbor in graph[current_point]:
        #         if neighbor not in unvisited_points.keys():
        #             continue
        #         if unvisited_points[neighbor] == 'inf':
        #             unvisited_points[neighbor] = current_cost + self.line_cost(line)
        #         new_weight = current_cost + self.line_cost(line)
        #         if new_weight <= unvisited_points[neighbor]:
        #             unvisited_points[neighbor] = new_weight
        #             shortest_path_points[neighbor] = current_point

        #     min_cost = 'inf'
        #     for unvisited_point in unvisited_points.keys():
        #         if  min_cost == 'inf' or (unvisited_points[unvisited_point] != 'inf' and\
        #                 unvisited_points[unvisited_point] < min_cost):
        #             min_cost = unvisited_points[unvisited_point]
        #             next_neighbor = unvisited_point
            
        #     current_cost = min_cost
        #     current_point = next_neighbor
                
        # path_point = dest
        # while path_point != source:
        #     new_point = shortest_path_points[path_point]
        #     shortest_path.insert(0,path_point)
        #     path_point = new_point
        # if (path_point != None):
        #     shortest_path.insert(0, path_point)

        # return shortest_path

    def extend(self):
        # TODO (student): Implement an extend logic.
        uniform_vector = self.x_random - self.x_near / self.planning_env.compute_distance((0, 0), self.x_random - self.x_near)
        return self.x_near + uniform_vector * self.step_size
        #pass

