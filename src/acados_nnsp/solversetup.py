from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from model import skid_model
import scipy.linalg
import numpy as np


def acados_settings(Tf, N, learned_dyn):
    # create render arguments
    ocp = AcadosOcp()

    # export model
    model, constraints = skid_model(learned_dyn)

    # define acados ODE
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.expr_f_impl
    model_ac.f_expl_expr = model.expr_f_expl
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    ocp.model = model_ac
    ocp.parameter_values = model.parameter_values

    # define constraint
    #model_ac.con_h_expr = constraints.expr

    # dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    nsbx = 1
    #nh = constraints.expr.shape[0]
    nh = 0
    nsh = nh
    ns = nsh + nsbx

    # discretization
    ocp.dims.N = N

    # set cost
    Q = np.diag([ 1e-1, 1e-1, 1e-8, 1e-3, 1e-3 ])

    R = np.eye(nu)
    R[0, 0] = 5e-3
    R[1, 1] = 5e-3

    Qe = np.diag([ 5e0, 5e0, 1e-8, 5e-3, 5e-3 ])
    #x0 = np.zeros((5, 1))
    #y_ref = np.array([0 0 0 0 0 270 0 0])
    #y_ref_e = np.array([0 0 0 0 0 270])
    xeW=45
    yeW=45
    thW=25
    vlW=1
    vrW=1
    sW=10
    aoW = 50
    aiBW = 10
    aoBW = 10
    alW=1
    arW=1
    W =  np.diag([xeW, yeW, thW, vlW, vrW, sW, alW, arW])
    #W =  np.diag([0, 0, 0, 0, 0, vlW, vrW])
    W_e = np.diag([xeW, yeW, thW, vlW, vrW, sW])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    unscale = N / Tf

    ocp.cost.W = W
    ocp.cost.W_e = W_e
    #ocp.cost.W = unscale * scipy.linalg.block_diag(Q, R)
    #ocp.cost.W_e = Qe / unscale

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[5, 0] = 1.0
    Vu[6, 1] = 1.0
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    ocp.cost.zl = 100 * np.ones((ns,))
    ocp.cost.zu = 100 * np.ones((ns,))
    ocp.cost.Zl = 1 * np.ones((ns,))
    ocp.cost.Zu = 1 * np.ones((ns,))

    # set intial references
    ocp.cost.yref = np.array([0, 0, 0, 0, 0, 270, 0, 0])
    ocp.cost.yref_e = np.array([0, 0, 0, 0, 0, 270])

    # setting constraints
    ocp.constraints.lbx = np.array([constraints.vmin, constraints.vmin])
    ocp.constraints.ubx = np.array([constraints.vmax, constraints.vmax])
    ocp.constraints.idxbx = np.array([3, 4])
    
    ocp.constraints.lbu = np.array([constraints.amin, constraints.amin])
    ocp.constraints.ubu = np.array([constraints.amax, constraints.amax])
    ocp.constraints.idxbu = np.array([0, 1])

    ocp.constraints.lsbx = np.zeros([nsbx])
    ocp.constraints.usbx = np.zeros([nsbx])
    ocp.constraints.idxsbx = np.array(range(nsbx))

    #ocp.constraints.lh = np.array(
    #    [
    #        -6,
    #    ]
    #)
    #ocp.constraints.uh = np.array(
    #    [
    #       6,
    #   ]
    #)
    ocp.constraints.lsh = np.zeros(nsh)
    ocp.constraints.ush = np.zeros(nsh)
    ocp.constraints.idxsh = np.array(range(nsh))

    # set intial condition
    ocp.constraints.x0 = model.x0

    # set QP solver and integration
    ocp.solver_options.tf = Tf
    #ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.levenberg_marquardt = 1e-10
    #ocp.solver_options.sim_method_num_stages = 4
    #ocp.solver_options.sim_method_num_steps = 3
    #ocp.solver_options.nlp_solver_step_length = 0.05
    ocp.solver_options.nlp_solver_max_iter = 140
    #ocp.solver_options.tol = 1e-4
    #ocp.solver_options.print_level = 0
    # ocp.solver_options.nlp_solver_tol_comp = 1e-1

    # create solver
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

    return constraints, model, acados_solver