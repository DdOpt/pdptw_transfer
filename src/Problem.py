class Request:
    def __init__(self):
        self.id = 0    # parcel id
        self.o = 0     # pickup point
        self.d = 0     # drop-off point
        self.e = 0     # the earliest service time from r_o
        self.l = 0     # the latest service time in r_d
        self.q = 0  # the quantity of parcel


class Instance:
    def __init__(self, nodes, transfers, requests, travel_time, parameters):
        self.T = transfers    # transfer point set
        self.nodes = nodes    # point set

        self.ROD = set() # od of each parcel
        self.q = {}  # key-value, od_of_parcel : quantity
        self.Request = []
        for idx in requests:
            r = Request()
            r.id = idx
            r.o, r.d, r.e, r.l, r.q = requests[idx]
            self.Request.append(r)
            self.q[r.o, r.d] = r.q
            self.ROD.add((r.o, r.d))

        # cost/distance/travel_time matrix
        self.c = travel_time

        self.Q = parameters["vehicle_capacity"]  # maximize capacity of vehicle
        self.s = parameters["vehicle_service_time"]   # service time
        self.t = parameters["parcel_transfer_time"]  # transfer operate time
        self.M = 100000  # a big integer
        self.FIX = parameters["vehicle_fixed_cost"]   # fixed cost to use a vehicle