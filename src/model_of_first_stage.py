import gurobipy as gp


class Model:
    def __init__(self, instance):
        self.instance = instance
        self.model = gp.Model("pickup and delivery with transfer")
        self.add_variables()
        self.add_constraints()
        self.set_objective()
        # self.model.optimize()

    def add_variables(self):
        self.x = {}
        for i in self.instance.nodes:
            for j in self.instance.nodes:
                if i == j:
                    continue
                self.x[i, j] = self.model.addVar(vtype=gp.GRB.BINARY, name=f"arc_{i}_{j}")

        self.y = {}
        for r in self.instance.Request:
            o, d = r.o, r.d
            for t in self.instance.nodes:
                if t == o or t == d:
                    continue
                self.y[o, t, d] = self.model.addVar(vtype=gp.GRB.BINARY, name=f"y_{o}_{t}_{d}")

        self.a = {}
        self.d = {}
        for i in self.instance.nodes:
            for j in self.instance.nodes:
                if i == j:
                    continue
                self.a[i, j] = self.model.addVar(lb=0.0, ub=2500, name=f"a_{i}_{j}")
                self.d[i, j] = self.model.addVar(lb=0.0, ub=2500, name=f"d_{i}_{j}")

    def set_objective(self):
        # self.model.setObjective(
        #     gp.quicksum(self.instance.c[i, j] * self.x[i, j] for i, j in self.x)   # travel cost
        #   + gp.quicksum(self.instance.q[o, d] * self.y[o, t, d] for o, t, d in self.y)  # transfer cost
        #   + gp.quicksum(1000 * self.x[i, j] for i, j in self.x), sense=gp.GRB.MINIMIZE)  # fixed vehicle cost
        # travel cost
        self.travel_cost = self.model.addVar(name=f"travel cost")
        self.model.addConstr(gp.quicksum(self.instance.c[i, j] * self.x[i, j] for i, j in self.x)
                             <= self.travel_cost)
        # transfer cost
        self.transfer_cost = self.model.addVar(name=f"transfer cost")
        self.model.addConstr(gp.quicksum(self.instance.q[o, d] * self.y[o, t, d] for o, t, d in self.y)
                             <= self.transfer_cost)
        # fixed vehicle cost
        self.fixed_vehicle_cost = self.model.addVar(name=f"fixed vehicle cost")
        self.model.addConstr(gp.quicksum(self.instance.FIX * self.x[i, j] for i, j in self.x) <= self.fixed_vehicle_cost)

        self.model.setObjective(self.travel_cost + self.transfer_cost + self.fixed_vehicle_cost,
                                sense=gp.GRB.MINIMIZE)

    def add_constraints(self):
        # constraint route choice
        for r in self.instance.Request:
            o, d = r.o, r.d
            self.model.addConstr(gp.quicksum(self.y[o, t, d] for t in self.instance.T if t != o and t != d)
                                 == 1-self.x[o, d], name=f"c2_request_{r.id}")
        # constraint
        for r in self.instance.Request:
            o, d = r.o, r.d
            for t in self.instance.T:
                if t == o or t == d:
                    continue
                # arcs x[o, t] and x[t, d] need to open when y[o, t, d] = 1
                self.model.addConstr(self.x[o, t] >= self.y[o, t, d], name=f"c3_{r.id}_t_{t}")
                self.model.addConstr(self.x[t, d] >= self.y[o, t, d], name=f"c4_{r.id}_t_{t}")

                # service time self.instance.s, transfer time self.instance.t
                # time constraint in r_o when y[o, t, d] = 1
                self.model.addConstr(r.e + self.instance.s
                                 - (1 - self.y[o, t, d]) * self.instance.M <= self.a[o, t], name=f"c9_{r.id}_{o}_{d}")

                # time constraint in r_d when y[o, t, d] = 1
                self.model.addConstr(self.d[t, d] + self.instance.s - (1 - self.y[o, t, d]) * self.instance.M <=
                                     r.l, name=f"c8_{r.id}_{t}")

                # time constraint in transfer point t when y[o, t, d] = 1
                self.model.addConstr(self.d[o, t] + self.instance.t - (1 - self.y[o, t, d])* self.instance.M <=
                                     self.a[t, d], name=f"c7_{r.id}_{t}")

            # time constraint for the parcels without transfer
            self.model.addConstr(r.e + self.instance.s
                                 - (1 - self.x[o, d])* self.instance.M <= self.a[o, d], name=f"c5_{r.id}_{o}_{d}")
            self.model.addConstr(self.d[o, d] + self.instance.s
                                - (1 - self.x[o, d]) * self.instance.M <= r.l, name=f"c6_{r.id}_{o}_{d}")
        # a natural time constraint
        for i, j in self.x:
            self.model.addConstr(self.a[i, j] + self.instance.c[i, j] <= self.d[i, j], name=f"c10_{i}_{j}")

        # capacity constraint of arc(i, j) when j is a transfer point
        for t in self.instance.T:
            for i in self.instance.nodes:
                expr = gp.LinExpr()
                if i == t:
                    continue
                if (i, t) in self.instance.ROD:
                    expr.add(self.x[i, t], self.instance.q[i, t])
                for j in self.instance.nodes:
                    if j == t or j == i:
                        continue
                    if (i, j) in self.instance.ROD:
                        expr.add(self.y[i, t, j], self.instance.q[i, j])

                self.model.addConstr(expr <= self.instance.Q * self.x[i, t], name=f"c11_t_{t}_{i}")
        # capacity constraint of arc(i, j) when i is a transfer point
        for t in self.instance.T:
            for j in self.instance.nodes:
                expr = gp.LinExpr()
                if j == t:
                    continue
                if (t, j) in self.instance.ROD:
                    expr.add(self.x[t, j], self.instance.q[t, j])
                for i in self.instance.nodes:
                    if i == t or i == j:
                        continue
                    if (i, j) in self.instance.ROD:
                        expr.add(self.y[i, t, j], self.instance.q[i,j])

                self.model.addConstr(expr <= self.instance.Q * self.x[t, j], name=f"c12_t_{t}_{j}")
        # capacity constraint of arc(i, j) when both i and j are transfer points
        for t1 in self.instance.T:
            for t2 in self.instance.T:
                expr = gp.LinExpr()
                if t1 == t2:
                    continue
                if (t1, t2) in self.instance.ROD:
                    expr.add(self.x[t1, t2], self.instance.q[t1, t2])

                for i in self.instance.nodes:
                    if i == t1 or i == t2:
                        continue
                    if (i, t2) in self.instance.ROD:
                        expr.add(self.y[i, t1, t2], self.instance.q[i, t2])

                    if (t1, i) in self.instance.ROD:
                        expr.add(self.y[t1, t2, i], self.instance.q[t1, i])

                self.model.addConstr(expr <= self.instance.Q * self.x[t1, t2], name=f"c13_t1_{t1}_t2_{t2}_{i}")


# instance = Instance()
# model = Model(instance)
# # model.model.write("model.lp")
#
# status = model.model.status
# if status == gp.GRB.OPTIMAL or (status == gp.GRB.TIME_LIMIT and model.model.SolCount > 0):
#     for (i, j) in model.x:
#         if model.x[i, j].x > 0.5:
#             print(i, j, "at", model.a[i, j].x, "dt", model.d[i,j].x)
#     for (i, t, j) in model.y:
#         if model.y[i, t, j].x > 0.5:
#             print(i, t, j)
# else:
#     print(status, model.model.SolCount)
    # model.model.computeIIS()
    # model.model.write("model.ilp")


