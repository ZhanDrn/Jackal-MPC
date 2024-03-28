from casadi import *
import numpy as np
import math

def skid_model(learned_dyn, N_obst):
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
    trackW=5.5
    # Constraints
    amin = -2
    amax = 2
    vmin = -2
    vmax = 2
    # States
    x = MX.sym('x')
    y = MX.sym('y')
    theta = MX.sym('theta')
    vl = MX.sym('vl')
    vr = MX.sym('vr')
    s = MX.sym('s')
    sym_x = vertcat(x, y, theta,vl,vr,s)
    x_dot = MX.sym('x_dot')
    y_dot = MX.sym('y_dot')
    theta_dot = MX.sym('theta_dot')
    vl_dot = MX.sym('vl_dot')
    vr_dot = MX.sym('vr_dot')
    s_dot = MX.sym('s_dot')
    sym_xdot = vertcat(x_dot, y_dot, theta_dot, vl_dot, vr_dot, s_dot)
    # Controls
    al = MX.sym('al')
    ar = MX.sym('ar')
    sym_u = vertcat(al, ar)
    # algebraic variables
    z = vertcat([])

    # parameters
    c = MX.sym('c')
    obsX = MX.sym('obsX',N_obst)
    obsY = MX.sym('obsY',N_obst)

    # Kinematics
    res_model = learned_dyn.approx(vertcat(2*sym_x[3],2*sym_x[4],sym_x[2]/math.pi,sym_u/2))
    #res_model = learned_dyn.approx(x)
    nnp = learned_dyn.sym_approx_params(order=1, flat=True)
    p = vertcat(nnp,c,obsX,obsY)
    #parameter_values = np.array([0])
    parameter_values = np.append(learned_dyn.approx_params(np.ones([5]), flat=True, order=1),np.zeros([1+2*N_obst]))
    vx = (alpha_l*yicr_r*vl - alpha_r*yicr_l*vr)/(yicr_r - yicr_l) 
    w = (alpha_l*vl - alpha_r*vr)/(yicr_r - yicr_l)
    vs = vx*cos(theta)+xicr*sin(theta)*w
    
    #vs = vx*cos(theta)+xicr*sin(theta)*w
    #expr_f_expl = vertcat(vx*cos(theta)+xicr*sin(theta)*w,
    #                      vx*sin(theta)-xicr*cos(theta)*w,
    #                      w,
    #                      al,
    #                      ar,)
    Expratio = 1
    LDratio = 1-Expratio
    expr_f_expl = vertcat((Expratio*(vx*cos(theta)+xicr*sin(theta)*w)+LDratio*0.5*res_model[0]) - vs*(1-c*y),
                          (Expratio*(vx*sin(theta)-xicr*cos(theta)*w)+LDratio*0.5*res_model[1]) - c*vs*x,
                          (Expratio*w+LDratio*2*res_model[2]) - c*vs,
                          (Expratio*al+LDratio*2*res_model[3]),
                          (Expratio*ar+LDratio*2*res_model[4]),
                          vs)
    #expr_f_expl = res_model
    expr_f_impl = sym_xdot - expr_f_expl
    obs = exp(-2*((obsX-x)**2+(obsY-y)**2))
    #obs = 1/(0.01+exp((obsX-x)**2+(obsY-y)**2))
    #obs_cost = sqrt(obs.T@obs)
    obs_cost = sum1(obs)

    cost_y = vertcat(x,y,theta,vl,vr,s,al,ar,obs_cost)
    #cost_y_e = vertcat(x,y,theta,vl,vr,s,al,ar,obs)
    cost_y_e = vertcat(x,y,theta,vl,vr,s)

    model_struct.y = cost_y
    model_struct.y_e = cost_y_e
    model_struct.x = sym_x
    model_struct.u = sym_u
    model_struct.xdot = sym_xdot
    model_struct.z = z
    model_struct.p = p
    model_struct.parameter_values = parameter_values
    model_struct.x0 = np.array([0, 0, 0, 0, 0, 0])
    model_struct.expr_f_expl = expr_f_expl
    model_struct.expr_f_impl = expr_f_impl
    model_struct.name = model_name
    constraints.amin = amin
    constraints.amax = amax
    constraints.vmin = vmin
    constraints.vmax = vmax

    # define constraints struct
    #constraints.v_x = Function("vx", [sym_x, sym_u], [vx])
    #constraints.expr = vertcat(vx)
    return model_struct, constraints