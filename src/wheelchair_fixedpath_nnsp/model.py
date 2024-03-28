from casadi import *
import numpy as np
import math

def skid_model(learned_dyn):
    #learned_dyn = MLP(5, 256, 5, 16, 'Tanh')
    model_struct = types.SimpleNamespace()
    constraints = types.SimpleNamespace()
    model_name = 'Skid_steered'
    #Model constants
    Ts = 0.05
    alpha_l = 1
    alpha_r = 1
    xicr= 0        #ideally icr is at origin
    yicr_l = 0.25 
    yicr_r = -0.25
    r_car = 0.3
    l_tr = 0.75
    #r_cw =
    dx_cw = 0.05
    dy_cw = 0.5
    trackW=5.5
    # Constraints
    awmin = -0.5
    awmax = 0.5
    amin = -0.5
    amax = 0.5
    wmin = -2
    wmax = 2
    vmin = -1
    vmax = 1
    # States
    x = MX.sym('x')
    y = MX.sym('y')
    theta = MX.sym('theta')
    vl = MX.sym('vl')
    w = MX.sym('w')
    s = MX.sym('s')
    cwL = MX.sym('cwL')
    cwR = MX.sym('cwR')
    sym_x = vertcat(x, y, theta,vl,w,s,cwL,cwR)
    x_dot = MX.sym('x_dot')
    y_dot = MX.sym('y_dot')
    theta_dot = MX.sym('theta_dot')
    vl_dot = MX.sym('vl_dot')
    w_dot = MX.sym('w_dot')
    s_dot = MX.sym('s_dot')
    cwL_dot = MX.sym('cwL_dot')
    cwR_dot = MX.sym('cwR_dot')
    sym_xdot = vertcat(x_dot, y_dot, theta_dot, vl_dot, w_dot, s_dot, cwL_dot, cwR_dot)
    # Controls
    al = MX.sym('al')
    aw = MX.sym('aw')
    sym_u = vertcat(al, aw)
    # algebraic variables
    z = vertcat([])

    # parameters
    c = MX.sym('c')
    #obsX = MX.sym('obsX',N_obst)
    #obsY = MX.sym('obsY',N_obst)

    # Kinematics
    res_model = learned_dyn.approx(vertcat(2*sym_x[3],2*sym_x[4],sym_x[2]/math.pi,sym_u/2))
    #res_model = learned_dyn.approx(x)
    nnp = learned_dyn.sym_approx_params(order=1, flat=True)
    p = vertcat(nnp,c)
    #parameter_values = np.array([0])
    parameter_values = np.append(learned_dyn.approx_params(np.ones([5]), flat=True, order=1),0)
    w_cwL = -(1/l_tr)*((vl-w*dy_cw)*sin(cwL)-w*dx_cw*cos(cwL))
    w_cwR = -(1/l_tr)*((vl+w*dy_cw)*sin(cwR)-w*dx_cw*cos(cwR))
    vs = vl*cos(theta)
    Expratio = 1
    LDratio = 1-Expratio
    expr_f_expl = vertcat(vl*cos(theta) - vs*(1-c*y),
                          vl*sin(theta) - c*vs*x,
                          w - c*vs,
                          al,
                          aw,
                          vs,
                          w_cwL,
                          w_cwR)
    #expr_f_expl = res_model
    expr_f_impl = sym_xdot - expr_f_expl
    #obs = exp(-2*((obsX-x)**2+(obsY-y)**2))
    #obs = 1/(0.01+exp((obsX-x)**2+(obsY-y)**2))
    #obs_cost = sqrt(obs.T@obs)
    #obs_cost = sum1(obs)

    cost_y = vertcat(x,y,theta,vl,w,s,al,aw,w_cwL,w_cwR)
    #cost_y_e = vertcat(x,y,theta,vl,vr,s,al,ar,obs)
    cost_y_e = vertcat(x,y,theta,vl,w,s)

    model_struct.y = cost_y
    model_struct.y_e = cost_y_e
    model_struct.x = sym_x
    model_struct.u = sym_u
    model_struct.xdot = sym_xdot
    model_struct.z = z
    model_struct.p = p
    model_struct.parameter_values = parameter_values
    model_struct.x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    model_struct.expr_f_expl = expr_f_expl
    model_struct.expr_f_impl = expr_f_impl
    model_struct.name = model_name
    constraints.awmin = awmin
    constraints.awmax = awmax
    constraints.amin = amin
    constraints.amax = amax
    constraints.wmin = wmin
    constraints.wmax = wmax
    constraints.vmin = vmin
    constraints.vmax = vmax

    # define constraints struct
    #constraints.v_x = Function("vx", [sym_x, sym_u], [vx])
    #constraints.expr = vertcat(vx)
    return model_struct, constraints