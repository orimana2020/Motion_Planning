#!/usr/bin/env python

import argparse, numpy, time

from MapEnvironment import MapEnvironment
from RRTPlanner import RRTPlanner
from RRTStarPlanner import RRTStarPlanner
from AStarPlanner import AStarPlanner

from IPython import embed

def main(planning_env, planner, start, goal):

    # Notify.
    #input('Press any key to begin planning')

    # Plan.
    plan = planner.Plan(start, goal)

    # TODO (student): Do not shortcut when comparing the performance of algorithms.
    # Comment this line out when collecting data over performance metrics.

    # Visualize the final path.
    planning_env.visualize_plan(plan)
    embed()


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')

    # parser.add_argument('-m', '--map', type=str, default='map1.txt',
    #                     help='The environment to plan on')    
    # parser.add_argument('-p', '--planner', type=str, default='rrt',
    #                     help='The planner to run (star, rrt, rrtstar)')
    # parser.add_argument('-s', '--start', nargs='+', type=int, required=True)
    # parser.add_argument('-g', '--goal', nargs='+', type=int, required=True)

    args = parser.parse_args()

    args.map = "map1.txt"
    args.planner = "rrtconnect"
    args.start = [1, 1]
    args.goal = [8, 7]

    # args.map = "map2.txt"
    # args.planner = "rrt"
    # args.start = [148, 321]
    # args.goal = [202, 106]

    # First setup the environment and the robot.
    planning_env = MapEnvironment(args.map, args.start, args.goal)

    # Next setup the planner
    if args.planner == 'astar':
        planner = AStarPlanner(planning_env)
    elif args.planner == 'rrt':
        planner = RRTPlanner(planning_env)
    elif args.planner == 'rrtconnect':
        planner = RRTStarPlanner(planning_env)
    else:
        print('Unknown planner option: %s' % args.planner)
        exit(0)

    main(planning_env, planner, args.start, args.goal)

