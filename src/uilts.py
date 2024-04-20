import pandas as pd


def read_demand_data(file_path):
    df = pd.read_csv(file_path)
    request_dict = {}
    for i in range(len(df)):
        id = int(df['id'][i])
        o, d = int(df["origin"][i][5:]),int(df["destination"][i][5:])
        e, l = int(df["start_time"][i]),int(df["end_time"][i])
        q = int(df["quantity"][i])
        request_dict[id] = [o, d, e, l, q]
    return request_dict
#print(read_demand_data("../data/demand_30.csv"))


def read_time_matrix(file_path):
    df = pd.read_csv(file_path)
    travel_time = {}
    for i in range(len(df)):
        x, y, time = int(df["name_x"][i][5:]), int(df["name_y"][i][5:]), int(df["time"][i])
        travel_time[x, y] = time
    return travel_time
#print(read_time_matrix("../data/time_matrix_30.csv"))


def read_parameter(file_path):
    df = pd.read_csv(file_path)
    parameter = {}
    for i in range(len(df)):
        parameter[df["parameter"][i]] = df["value"][i]
    return parameter
#print(read_parameter("../data/parameter.csv"))


def read_node_data(file_path):
    df = pd.read_csv(file_path)
    nodes = set()
    transfers = set()
    for i in range(len(df)):
        nodes.add(int(df["name"][i][5:]))
        if df["transfer"][i]:
            transfers.add(int(df["name"][i][5:]))
    return nodes, transfers
#print(read_node_data("../data/node_30.csv"))


def get_parcel_flow(x, y, instance):
    parcel_to_od, od_to_parcel = {}, {}
    for r in instance.Request:
        parcel_to_od[r.id] = (r.o, r.d)
        od_to_parcel[r.o, r.d] = r.id

    arcs, vehicles = {}, {}
    parcel_without_transfer = {}
    vehicle_parcel = {}
    veh = 1
    for (i, j) in x:
        arcs[i, j] = veh
        vehicles[veh] = (i, j)
        if (i, j) in od_to_parcel:
            parcel_without_transfer[od_to_parcel[i, j]] = (i, j)
            vehicle_parcel[veh] = [od_to_parcel[i, j]]
        veh += 1
    parcel_with_transfer = {}
    for (o, t, d) in y:
        vehicle_parcel[arcs[o, t]].append(od_to_parcel[o, d])
        vehicle_parcel[arcs[t, d]].append(od_to_parcel[o, d])
        parcel_with_transfer[od_to_parcel[o, d]] = (o, t, d)

    for parcel in parcel_with_transfer:
        print(parcel, parcel_with_transfer[parcel])
    for parcel in parcel_without_transfer:
        print(parcel, parcel_without_transfer[parcel])
    print(vehicle_parcel)
    return vehicle_parcel, parcel_without_transfer, parcel_with_transfer, vehicles, arcs


def pre_solve(vehicle_parcel, instance, arcs, parcel_with_transfer):
    e, l = {}, {}
    for veh in vehicle_parcel:
        max_e = 0
        min_l = float("inf")
        for parcel in vehicle_parcel[veh]:
            if instance.Request[parcel].e > max_e:
                max_e = instance.Request[parcel].e
            if instance.Request[parcel].l < min_l:
                min_l = instance.Request[parcel].l
        e[veh] = max_e
        l[veh] = min_l
    paris = []
    for parcel in parcel_with_transfer:
        paris.append((arcs[parcel_with_transfer[parcel][0], parcel_with_transfer[parcel][1]],
                      arcs[parcel_with_transfer[parcel][1], parcel_with_transfer[parcel][2]]))
    return e, l, paris