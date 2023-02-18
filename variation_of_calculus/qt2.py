import casadi as ca

# state symbolic variables
x = ca.SX.sym('w')
obj = ca.exp(0.2*x)*ca.sin(x)

g = [] # constraints 
P = [] # problem parameters 

variables = x # decision variable 

nlp_prob = {
    'f': obj,
    'x': variables,
    'g': g,
    'p': P
}

opts = {
    'ipopt': {
        'max_iter': 2000,
        'print_level': 0,
        'acceptable_tol': 1e-8,
        'acceptable_obj_change_tol': 1e-6
    },
    'print_time': 0
}

solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

lbx = ca.DM.zeros((1, 1))
ubx = ca.DM.zeros((1, 1))
lbx[0:1: 1] = 0     # X lower bound
ubx[0:1: 1] = 4*ca.pi      # X upper bound

lbg = ca.DM.zeros((1, 1))
ubg = ca.DM.zeros((1, 1))
lbg[0:1: 1] = -ca.inf     # constraints lower bound
ubg[0:1: 1] = ca.inf      # constraints upper bound

args = {
    'lbg': lbg,  # constraints lower bound
    'ubg': ubg,  # constraints upper bound
    'lbx': lbx,
    'ubx': ubx,
    'p':[],
    'x0': 4.0
}

if __name__ == '__main__':
    sol = solver(x0=args['x0'], lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'], p=args['p'])
    min_value = sol['x'].full()
    print(min_value)
        
        
      