import math
import utility as utility
import loader as loader
import numpy as np


def main():

    # Paths to the data and solution files.
    vrp_file = 'n32-k5.vrp' #"n80-k10.vrp"
    sol_file = 'n32-k5.sol' #"n80-k10.sol"

    # Loading the VRP data file.
    px, py, demand, capacity, depot = loader.load_data(vrp_file)

    # Displaying to console the distance and visualizing the optimal VRP solution.
    vrp_best_sol = loader.load_solution(sol_file)
    best_distance = utility.calculate_total_distance(vrp_best_sol, px, py, depot)
    print("Best VRP Distance:", best_distance)
    utility.visualise_solution(vrp_best_sol, px, py, depot, "Optimal Solution")

    # Executing and visualizing the nearest neighbour VRP heuristic.
    nnh_solution = nearest_neighbour_heuristic(px, py, demand, capacity, depot)
    nnh_distance = utility.calculate_total_distance(nnh_solution, px, py, depot)
    print("Nearest Neighbour VRP Heuristic Distance:", nnh_distance)
    utility.visualise_solution(nnh_solution, px, py, depot, "Nearest Neighbour Heuristic")

    # Executing and visualizing the saving VRP heuristic.    
    sh_solution = savings_heuristic(px, py, demand, capacity, depot)
    sh_distance = utility.calculate_total_distance(sh_solution, px, py, depot)
    print("Saving VRP Heuristic Distance:", sh_distance)
    utility.visualise_solution(sh_solution, px, py, depot, "Savings Heuristic")


def nearest_neighbour_heuristic(px, py, demand, capacity, depot):

    """
    Algorithm for the nearest neighbour heuristic to generate VRP solutions.

    :param px: List of X coordinates for each node.
    :param py: List of Y coordinates for each node.
    :param demand: List of each nodes demand.
    :param capacity: Vehicle carrying capacity.
    :param depot: Depot.
    :return: List of vehicle routes (tours).
    """

    routes = []
    remainingNodes = [i+1 for i in range(len(px)-1)] # list of nodes to visit - exclude depot

    while len(remainingNodes) > 0:
        route = []
        totalDemand = 0
        currentNode = depot
        nextNode = -1
        while totalDemand < capacity:
            if len(remainingNodes) == 0:
                break

            # find nearest node
            minDist = float('inf')
            for node in remainingNodes:
                dist = utility.calculate_euclidean_distance(px,py,currentNode,node)
                # check if this node exceeds the capacity, if so, skip it
                if(dist < minDist and totalDemand + demand[node] <= capacity):
                    minDist = dist
                    nextNode = node
            
            if(nextNode == -1): # no feasible node found
                break

            route.append(nextNode)
            totalDemand += demand[nextNode]
            remainingNodes.remove(nextNode)
            currentNode = nextNode
            nextNode = -1 # set next node to unknown
            
        routes.append(route)
    
    print("NN Routes:")
    print(routes)

    return routes


def savings_heuristic(px, py, demand, capacity, depot):

    """
    Algorithm for Implementing the savings heuristic to generate VRP solutions.

    :param px: List of X coordinates for each node.
    :param py: List of Y coordinates for each node.
    :param demand: List of each nodes demand.
    :param capacity: Vehicle carrying capacity.
    :param depot: Depot.
    :return: List of vehicle routes (tours).
    """

    # TODO - Implement the Saving Heuristic to generate VRP solutions.
    routes = []
    nodes = [i+1 for i in range(len(px)-1)] # list of nodes - exclude depot

    # initiliase routes
    for node in nodes:
        routes.append([node])

    canMerge = True
    while canMerge:
        merges = []
        for routeA in routes:
            for routeB in routes:
                if routeA == routeB:
                    continue
                # calculate savings
                merge = routeA + routeB
                costA = utility.calculate_total_distance([routeA], px, py, depot)
                costB = utility.calculate_total_distance([routeB], px, py, depot)
                costMerge = utility.calculate_total_distance([merge], px, py, depot)
                savings = costA + costB - costMerge
                # check if merge is feasible to add in feasible merges
                totalDemand = 0
                for node in merge:
                    totalDemand += demand[node]
                if totalDemand <= capacity:
                    merges.append((routeA, routeB, savings))
        
        # if there are no feasible merges, stop
        if len(merges) == 0:
            canMerge = False
            break

        # select the highest savings for the feasible merges
        nextMerge = max(merges, key=lambda item: item[2])
        # remove the routes that are being merged
        routes.remove(nextMerge[0])
        routes.remove(nextMerge[1])
        # add the merged route
        routes.append(nextMerge[0] + nextMerge[1])
        #print("Merged Routes:", routes)

    print("SH Routes:")
    print(routes)
    return routes


if __name__ == '__main__':
    main()
