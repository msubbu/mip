from __future__ import print_function

__author__ = 'Madhu'

import math
from collections import namedtuple
from gurobipy import *

#euclidian distance
def dist(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

Point = namedtuple("Point", ['x', 'y'])
Facility = namedtuple("Facility", ['index', 'setup_cost', 'capacity', 'location'])
Customer = namedtuple("Customer", ['index', 'demand', 'location'])

def get_facility_location(input_data):

    lines = input_data.split('\n')

    parts = lines[0].split()
    facility_count = int(parts[0])
    customer_count = int(parts[1])

    facilities = []
    for i in range(1, facility_count+1):
        parts = lines[i].split()
        facilities.append(Facility(i-1, float(parts[0]), int(parts[1]), Point(float(parts[2]), float(parts[3])) ))

    customers = []
    for i in range(facility_count+1, facility_count+1+customer_count):
        parts = lines[i].split()
        customers.append(Customer(i-1-facility_count, int(parts[0]), Point(float(parts[1]), float(parts[2]))))

    demands = [x.demand for x in customers]

    capacities = [x.capacity for x in facilities]

    fixed_costs = [x.setup_cost for x in facilities]

    model = Model("FacilityLocation")

    openFacilities = []

    #first component of objective function: fixed costs
    for f in facilities:
        openFacilities.append(model.addVar(vtype=GRB.BINARY,obj=fixed_costs[f.index],name="open%d" % f.index))

    #second component of objective function: distances from facilities to customers
    cust_to_facility = []
    for c in customers:
        cust_to_facility.append([])
        for f in facilities:
            cust_to_facility[c.index].append(model.addVar(vtype=GRB.BINARY,obj=dist(f.location,c.location),name="cust_to_facility_f%d_c_%d" % (f.index,c.index)))

    model.modelSense = GRB.MINIMIZE

    model.update()

    #assign exactly one customer per facility
    for c in customers:
        model.addConstr(quicksum(cust_to_facility[c.index][f.index] for f in facilities) == 1, "one_cust_per_facility_c%d"%c.index)

    #capacity of facility must not exceed sum of all customer demands
    for f in facilities:
        model.addConstr(quicksum(demands[c.index]*cust_to_facility[c.index][f.index] for c in customers) <= capacities[f.index] * openFacilities[f.index], "facility_capacity_f%d"%f.index)

    model.params.method = 2
    model.setParam(GRB.param.MIPGap,0.05)

    model.optimize()


    for f in openFacilities:
        print(f.getAttr(GRB.Attr.VarName)+" %d"%f.x)

    for i,cf in enumerate(cust_to_facility):
        for j,k in enumerate(cf):
            if k.x > 0:
                print("Customer %d assigned to facility %d"%(i,j))

    sol=[]
    for cf in cust_to_facility:
        sol.append(next((i for i,k in enumerate(cf) if k.x > 0), None))

    result = "%d 0\n"%model.objVal
    result += ' '.join(map(str,sol))

    model.write("./models/f%d_c%d.lp"%(facility_count,customer_count))



    return result

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        res = get_facility_location(input_data)
        print(res)

        with open(file_location+".sol","w") as res_file:
            res_file.write(res)

    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python ssfacilityloc.py ./data/fl_16_2)')