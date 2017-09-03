from __future__ import print_function

__author__ = 'Madhu'

#core model adopted from Gurobi's Tanker routing problem here - http://examples.gurobi.com/routing-tanker-trucks/

import math
from collections import namedtuple
from gurobipy import *
import sys

Customer = namedtuple("Customer", ['index', 'demand', 'x', 'y'])

#Eucledian distances
def dist(c1, c2):
    return math.sqrt((c1.x - c2.x)**2 + (c1.y - c2.y)**2)

def get_routes(input_data):

    lines = input_data.split('\n')
    parts = lines[0].split()
    location_count = int(parts[0])
    vehicle_count = int(parts[1])
    vehicle_capacity = int(parts[2])

    locations = []

    for i in range(1, location_count+1):
        line = lines[i]
        parts = line.split()
        locations.append(Customer(i-1, int(parts[0]), float(parts[1]), float(parts[2])))

    depot = locations[0]
    customers = locations[1:]

    sites = range(location_count)

    model = Model('Vehicle Routing')

    #objective: minimize total miles driven
    x = {}
    for i in sites:
        for j in sites:
            x[i,j] = model.addVar(vtype=GRB.BINARY)

    model.setObjective(quicksum(dist(i,j) * x[i.index,j.index] for i in locations for j in locations if i != j))

    #u must be more than the client's demand, but less than the capacity of the vehicle
    u = {}
    for i in customers:
        u[i.index-1] = model.addVar(lb = i.demand, ub = vehicle_capacity)

    model.update()

    #enter a customer location exactly once
    for j in customers:
        model.addConstr(quicksum(x[i,j.index] for i in sites if i != j.index) == 1)

    #leave a customer location exactly once
    for i in customers:
        model.addConstr(quicksum(x[i.index,j] for j in sites if i.index != j) == 1)

    #for the first customer, force u == q
    for c in customers:
        model.addConstr(u[c.index - 1] <= vehicle_capacity + (c.demand - vehicle_capacity) * x[0,c.index])

    #eliminate subtours
    for i in customers:
        for j in customers:
            if i != j:
                model.addConstr(u[i.index -1] - u[j.index - 1] + vehicle_capacity * x[i.index,j.index] <= vehicle_capacity - j.demand)

    model.setParam(GRB.param.MIPGap,0.05)

    model.update()

    model.optimize()

    #number of vehicles needed = out degree of depot
    first_stops = [c.index for c in customers if x[depot.index,c.index].x > 0]

    #returns the next out neighbor for a given node
    def get_out_neighbor(adjMatrix,node):
        for k in locations:
            if adjMatrix[node,k.index].x > 0:
                return k.index

    #build eulerian tour from adjacency matrix given the first stop
    def assemble_tour(path_so_far,adjMatrix,depot_location):
        next = get_out_neighbor(adjMatrix,path_so_far[-1])
        if next == depot_location:
            return path_so_far + [next]
        else:
            return assemble_tour(path_so_far + [next],adjMatrix,depot_location)

    #build all eulerian tours
    tours = [assemble_tour([depot.index] + [i],x,depot.index) for i in first_stops]

    result = "%d 0\n"%model.objVal
    for tour in tours:
        result += ' '.join(map(str,tour))+"\n"

    model.write("./models/c%d_v%d.lp"%(len(customers),vehicle_count))

    return(result)


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        res = get_routes(input_data)
        print(res)

        with open(file_location+".sol","w") as res_file:
            res_file.write(res)
    else:

        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/vrp_5_4_1)')

