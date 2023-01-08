import numpy
from IPython import embed
from matplotlib import pyplot as plt

class MapEnvironment(object):
    
    def __init__(self, mapfile, start, goal):

        # Obtain the boundary limits.
        # Check if file exists.
        self.map = numpy.loadtxt(mapfile)
        self.xlimit = [1, self.map.shape[1]]
        self.ylimit = [1, self.map.shape[0]]

        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits')
            exit(0)

        # Display the map
        plt.imshow(self.map, interpolation='nearest')

        self.goal = goal

    def compute_distance(self, start_config, end_config):
        
        #
        # TODO: Implement a function which computes the distance between
        # two configurations.
        #
        return numpy.sqrt(numpy.sum(numpy.power(numpy.array(start_config) - numpy.array(end_config), 2)))
        pass


    def state_validity_checker(self, config):

        #
        # TODO: Implement a state validity checker
        # Return true if valid.
        #
        new_config = [int(numpy.ceil(config[0] - 0.5)), int(numpy.ceil(config[1] - 0.5)) ]
        boundary_check_x = new_config[0] >= self.xlimit[0] and new_config[0] <= self.xlimit[1] - 1
        boundary_check_y = new_config[1] >= self.ylimit[0] and new_config[1]  <= self.ylimit[1] - 1
        if (not(boundary_check_x and boundary_check_y)):
            return False
        #Maybe should change!
        #config = [int(numpy.ceil(config[0] - 0.5)), int(numpy.ceil(config[1] - 0.5)) ]
        collision_check = self.map[new_config[1], new_config[0]] == 0
        return collision_check
        #return True

    def edge_validity_checker(self, config1, config2):

        #
        # TODO: Implement an edge validity checker
        #
        #
        config2 = numpy.array(config2)
        config1 = numpy.array(config1)
        relaxtion_factor = 5
        n = max(self.xlimit[1], self.ylimit[1]) + relaxtion_factor
        #n = int(self.compute_distance(config1, config2) * n)

        x_vals = numpy.linspace(config1[0], config2[0], n).reshape(1, n)
        y_vals = numpy.linspace(config1[1], config2[1], n).reshape(1, n)
        points_to_check = numpy.vstack((x_vals, y_vals))
        points_to_check = numpy.hstack((points_to_check, config2.reshape(2, 1)))
        for i in range(points_to_check.shape[1]):
            #print([points_to_check[0, i], points_to_check[1, i]])
            if not self.state_validity_checker([points_to_check[0, i], points_to_check[1, i]]):
                return False

        # for i in range(points_to_check.shape[1]):
        #     plot_x = [points_to_check[1, i]]
        #     plot_y = [points_to_check[0, i]]
        #     plt.plot(plot_y, plot_x,'-bo',markersize=5)
        return True
        pass

    def compute_heuristic(self, config):
        
        #
        # TODO: Implement a function to compute heuristic.
        #
        return numpy.sqrt(numpy.sum(numpy.power(numpy.array(config) - numpy.array(self.goal), 2)))
        pass

    def visualize_plan(self, plan):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        input plan should be in [x, y] convention.
        '''
        plt.imshow(self.map, interpolation='nearest')
        for i in range(numpy.shape(plan)[0] - 1):
            x = [plan[i,0], plan[i+1, 0]]
            y = [plan[i,1], plan[i+1, 1]]
            plt.plot(x, y, 'k')
        plt.show()