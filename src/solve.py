from Problem import *
from model_of_first_stage import *
from model_of_second_stage import *
from uilts import *


num_of_nodes = 50
requests = read_demand_data(f"../data/demand_{num_of_nodes}.csv")
travel_time = read_time_matrix(f"../data/time_matrix_{num_of_nodes}.csv")
parameters = read_parameter(f"../data/parameter.csv")
nodes, transfers = read_node_data(f"../data/node_{num_of_nodes}.csv")

instance = Instance(nodes, transfers, requests, travel_time, parameters)


first_stage_model = Model(instance)
first_stage_model.model.setParam('TimeLimit', 120)
first_stage_model.model.optimize()
x = set()
y = set()
status = first_stage_model.model.status
if status == gp.GRB.OPTIMAL or (status == gp.GRB.TIME_LIMIT and first_stage_model.model.SolCount > 0):
    print("first stage cost")
    print("travel_cost, ", first_stage_model.travel_cost.x)
    print("transfer_cost, ", first_stage_model.transfer_cost.x)
    print("fix vehicle_cost, ", first_stage_model.fixed_vehicle_cost.x)
    for (i, j) in first_stage_model.x:
        if first_stage_model.x[i, j].x > 0.5:
            x.add((i, j))

    for (i, t, j) in first_stage_model.y:
        if first_stage_model.y[i, t, j].x > 0.5:
            y.add((i, t, j))

    vehicle_parcel, parcel_without_transfer, parcel_with_transfer, vehicles, arcs \
        = get_parcel_flow(x, y, instance)
    e, l, pairs = pre_solve(vehicle_parcel, instance, arcs, parcel_with_transfer)

    second_stage_model = Schedule(e, l, pairs, instance, vehicles, arcs)
    second_stage_model.model.setParam('TimeLimit', 120)
    # second_stage_model.model.optimize()

else:
    print(status, first_stage_model.model.SolCount)