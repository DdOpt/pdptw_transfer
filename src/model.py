import gurobipy as gp


class Model:
    def __init__(self, instance):
        self.instance = instance
        self.model = gp.Model("pickup and delivery with transfer")

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
                self.y[o,t, d] = self.model.addVar(vtype=gp.GRB.BINARY, name=f"transfer_{t}_arc_{o}_{d}")

        self.a = {}
        self.d = {}
        for i in self.instance.nodes:
            for j in self.instance.nodes:
                if i == j:
                    continue
                self.a[i, j] = self.model.addVar(lb=0.0, ub=2500, name=f"start into arc({i}, {j})")
                self.d[i, j] = self.model.addVar(lb=0.0, ub=2500, name=f"leave from arc({i}, {j})")

    def set_objective(self):
        self.model.setObjective(
            gp.quicksum(self.instance.c[i, j] * self.x[i, j] for i, j in self.x)
          + gp.quicksum(self.instance.r.q[o, d] * self.y[o, t, d] for o, t, d in self.y)
          + gp.quicksum(1000 * self.x[i, j] for i, j in self.x), sense=gp.GRB.MINIMIZE)

    def add_constraints(self):
        for r in self.instance.Request:
            o, d = r.o, r.d
            self.model.addConstr(gp.quicksum(self.y[o, t, d] for t in self.instance.T if t != o and t != d)
                                 == 1-self.x[o, d], name=f"c2_request_{r}")

        for r in self.instance.Request:
            o, d = r.o, r.d
            for t in self.instance.T:
                if t == o or t ==d:
                    continue
                self.model.addConstr(self.x[o, t] >= self.y[o, t, d], name=f"c3_{r}_t_{t}")
                self.model.addConstr(self.x[t, d] >= self.y[o, t, d], name=f"c4_{r}_t_{t}")

                self.model.addConstr(self.instance.e[r] + self.instance.s
                                 - (1 - self.y[o, t, d])* self.instance.M <= self.a[o, t], name=f"c9_{r}_{o}_{d}")

                self.model.addConstr(self.d[t, d] + self.instance.s - (1 - self.y[o, t, d])* self.instance.M <=
                                     self.instance.l[r], name=f"c8_{r}_{t}")

                self.model.addConstr(self.d[o, t] + self.instance.t - (1 - self.y[o, t, d])* self.instance.M <=
                                     self.a[t, d], name=f"c7_{r}_{t}")

            self.model.addConstr(self.instance.e[r] + self.instance.s
                                 - (1 - self.x[o, d])* self.instance.M <= self.a[o, d], name=f"c5_{r}_{o}_{d}")

            self.model.addConstr(self.d[o, d] + self.instance.s
                                - (1 - self.x[o, d]) * self.instance.M <= self.instance.e[r], name=f"c6_{r}_{o}_{d}")

            for i, j in self.x:
                self.model.addConstr(self.a[i, j] + self.instance.c[i, j] <= self.d[i, j], name=f"c10_{i}_{j}")






