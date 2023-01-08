import sys
import time
import numpy

class AStarPlanner(object):    
    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.nodes = dict()

    def Plan(self, start_config, goal_config):

        plan = []

        # TODO (student): Implement your planner here.
        epsilon = 1
        start_config = tuple(start_config)
        goal_config = tuple(goal_config)

        #plan.append(start_config)
        #plan.append(goal_config)

        closed_set = set()
        open_set = set()
        open_set.add(start_config)
        came_from = {}
        g_score = {}
        f_score = {}

        g_score[start_config] = 0
        f_score[start_config] = g_score[start_config] + \
            self.planning_env.compute_heuristic(start_config) * epsilon

        print(self.nodes)
        while(len(open_set) > 0):
            min_config = min(f_score, key=f_score.get)
            if (min_config == goal_config):
                #reconstruct path
                current = goal_config
                while current != start_config:
                    plan.insert(0, current)
                    current = came_from[current]
                break
            open_set.remove(min_config)
            del f_score[min_config] #is it ok??
            closed_set.add(min_config)
            #Getting_neighbors:
            neighbors = []
            for i in [-1, 0 ,1]:
                for j in [-1, 0, 1]:
                    if i == 0 and j == 0:
                        continue
                    new_config = (min_config[0] + i, min_config[1] + j)
                    if self.planning_env.state_validity_checker(new_config):
                        neighbors.append((min_config[0] + i, min_config[1] + j))
            
            for neighbor in neighbors:
                tentative_g_score = g_score[min_config] + self.planning_env.compute_distance(min_config,
                    neighbor)
                if neighbor in closed_set:
                    continue
                if neighbor not in open_set or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = min_config
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + \
                     self.planning_env.compute_heuristic(neighbor) * epsilon
                    if neighbor not in open_set:
                        open_set.add(neighbor)



        return numpy.array(plan)

