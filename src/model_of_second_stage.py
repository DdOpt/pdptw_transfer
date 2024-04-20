import gurobipy as gp


class Schedule:
    def __init__(self, e, l, pairs, instance, vehicles, arcs):
        self.instance = instance
        self.vehicles = vehicles
        self.e = e
        self.l = l
        self.pairs = pairs
        self.model = gp.Model("schedule model")
        self.create_variables()
        self.build_constraints()
        self.set_objective()
        # self.model.optimize()

    def create_variables(self):
        self.keys = {**self.e, **{0:(0, 0)}}
        self.x = {}
        for i in self.keys:
            for j in self.keys:
                if i ==j:
                    continue
                self.x[i, j] = self.model.addVar(vtype=gp.GRB.BINARY, name=f"x_{i}")

        self.a = {}
        self.d = {}
        for i in self.keys:
            if i != 0:
                self.a[i] = self.model.addVar(lb=self.e[i], name=f"a_{i}")
                self.d[i] = self.model.addVar(ub=self.l[i], name=f"d_{i}")

    def build_constraints(self):
        for i in self.keys:
            if i == 0:
                continue
            self.model.addConstr(self.a[i] + 2 * self.instance.s + self.instance.c[self.vehicles[i]]
                                 <= self.d[i], name=f"c1_{i}")

        for (i, j) in self.pairs:
            self.model.addConstr(self.d[i] + self.instance.t - 2 * self.instance.s
                                 <= self.a[j], name=f"c2_{i}_{j}")

        for i in self.e:
            self.model.addConstr(gp.quicksum(self.x[i, j] for j in self.keys if i != j) == 1, name=f"c3_{i}")
            self.model.addConstr(gp.quicksum(self.x[j, i] for j in self.keys if i != j) == 1, name=f"c4_{i}")

        for (i, j) in self.x:
            if i == 0 or j == 0:
                continue
                # self.model.addConstr(self.d[i] - (1 - self.x[i, j]) * self.instance.M
                #                      <= self.a[j], name=f"c5_{i}_{j}")
            else:
                self.model.addConstr(self.d[i] + self.instance.c[self.vehicles[i][1], self.vehicles[j][0]]
                                    - (1 - self.x[i, j]) * self.instance.M
                                    <= self.a[j], name=f"c6_{i}_{j}")

    def set_objective(self):
        self.model.setObjective(
            1000 * gp.quicksum(self.x[0, i] for i in self.e) +
            gp.quicksum(self.instance.c[self.vehicles[i]] for i in self.e) +
            gp.quicksum(self.x[i, j] * self.instance.c[self.vehicles[i][1], self.vehicles[j][0]]
                        for (i, j) in self.x if i != 0 and j != 0), sense=gp.GRB.MINIMIZE)
