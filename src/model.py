import gurobipy as gp


class Request:
    def __init__(self):
        self.id = 0
        self.o = 0
        self.d = 0
        self.e = 0
        self.q = 0
        self.l = 2000


class Instance:
    def __init__(self):
        self.T = {2}
        self.nodes = {i for i in range(1, 4)}
        self.ROD = {(1, 2), (1, 3), (2, 3), (3, 2)}
        self.q = {(1, 2): 50, (1, 3): 50, (2, 3): 50, (3, 2): 100}
        self.Request = []
        self.Q = 100
        self.s = 100
        self.t = 400
        self.M = 10000000
        self.c = \
            {(1, 1): 0, (1, 2): 200, (1, 3): 100,
            (2, 1): 200, (2, 2): 0, (2, 3): 100,
            (3, 1): 100, (3, 2): 100, (3, 3): 0}
        num = 0
        for (o, d) in self.q:
            r = Request()
            r.id = num
            r.q = self.q[o, d]
            r.o, r.d = o, d
            self.Request.append(r)
            num += 1
        # print(self.Request)


class Model:
    def __init__(self, instance):
        self.instance = instance
        self.model = gp.Model("pickup and delivery with transfer")
        self.add_variables()
        self.add_constraints()
        self.set_objective()
        self.model.optimize()

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
        self.model.setObjective(
            gp.quicksum(self.instance.c[i, j] * self.x[i, j] for i, j in self.x)
          + gp.quicksum(self.instance.q[o, d] * self.y[o, t, d] for o, t, d in self.y)
          + gp.quicksum(1000 * self.x[i, j] for i, j in self.x), sense=gp.GRB.MINIMIZE)

    def add_constraints(self):
        for r in self.instance.Request:
            o, d = r.o, r.d
            self.model.addConstr(gp.quicksum(self.y[o, t, d] for t in self.instance.T if t != o and t != d)
                                 == 1-self.x[o, d], name=f"c2_request_{r.id}")

        for r in self.instance.Request:
            o, d = r.o, r.d
            for t in self.instance.T:
                if t == o or t == d:
                    continue
                self.model.addConstr(self.x[o, t] >= self.y[o, t, d], name=f"c3_{r.id}_t_{t}")
                self.model.addConstr(self.x[t, d] >= self.y[o, t, d], name=f"c4_{r.id}_t_{t}")

                self.model.addConstr(r.e + self.instance.s
                                 - (1 - self.y[o, t, d])* self.instance.M <= self.a[o, t], name=f"c9_{r.id}_{o}_{d}")

                self.model.addConstr(self.d[t, d] + self.instance.s - (1 - self.y[o, t, d]) * self.instance.M <=
                                     r.l, name=f"c8_{r.id}_{t}")

                self.model.addConstr(self.d[o, t] + self.instance.t - (1 - self.y[o, t, d])* self.instance.M <=
                                     self.a[t, d], name=f"c7_{r.id}_{t}")

            self.model.addConstr(r.e + self.instance.s
                                 - (1 - self.x[o, d])* self.instance.M <= self.a[o, d], name=f"c5_{r.id}_{o}_{d}")

            self.model.addConstr(self.d[o, d] + self.instance.s
                                - (1 - self.x[o, d]) * self.instance.M <= r.l, name=f"c6_{r.id}_{o}_{d}")

            for i, j in self.x:
                self.model.addConstr(self.a[i, j] + self.instance.c[i, j] <= self.d[i, j], name=f"c10_{i}_{j}")

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


instance = Instance()
model = Model(instance)
model.model.write("model.lp")
if model.model.status == gp.GRB.OPTIMAL:
    for (i, j) in model.x:
        if model.x[i,j].x > 0.5:
            print(i, j)

    for (i, t, j) in model.y:
        if model.y[i, t, j].x > 0.5:
            print(i, t, j)
else:
    model.model.computeIIS()
    model.model.write("model.ilp")
