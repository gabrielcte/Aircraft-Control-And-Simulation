import scipy.optimize
import numpy as np
from pathlib                import Path
import jsbsim

def trim_optimization(fdm, ic, design_vector, x0, debug_level,
         cost=None, eq_constraints=None, tol=1e-5, ftol=None, show=False, **kwargs):

        
    if cost is None:
        def cost(fdm):
            # compute cost, force moment balance
            theta = fdm['attitude/theta-rad']
            udot  = fdm['accelerations/udot-ft_sec2']
            vdot  = fdm['accelerations/vdot-ft_sec2']
            wdot  = fdm['accelerations/wdot-ft_sec2']
            pdot  = fdm['accelerations/pdot-rad_sec2']
            qdot  = fdm['accelerations/qdot-rad_sec2']
            rdot  = fdm['accelerations/rdot-rad_sec2']
            # (theta < 0) term prevents nose down trim
            return udot**2 + vdot**2 + wdot**2 + pdot**2 + qdot**2 + rdot**2 + + 1e-3*(theta < 0)

    # cost function for trimming
    def eval_fdm_func(xd, fdm, ic, fdm_func):
        # set initial condition
        for var in ic.keys():
            fdm[var] = ic[var]

        # set design vector
        for i, var in enumerate(design_vector):
            fdm[var] = xd[i]
        
        # trim propulsion
        prop = fdm.get_propulsion()
        prop.init_running(0)        

        # set initial conditions
        fdm.run_ic()

        return fdm_func(fdm)

    # setup constraints
    constraints = []

    if eq_constraints is None:
        eq_constraints = []

    for con in eq_constraints:
        constraints.append({
            'type': 'eq',
            'fun': eval_fdm_func,
            'args': (fdm, ic, con)
        })
    
    options = {'maxiter': 100}                              # Increase maxiter to 100 iterations

    # solve
    res = scipy.optimize.minimize(
        fun  = eval_fdm_func,
        args = (fdm, ic, cost), x0=x0, 
        constraints=constraints, **kwargs,
        options=options)

    # update ic
    for i, var in enumerate(design_vector):
        ic[var] = res['x'][i]

    if not res['success'] or ftol is not None and abs(res['fun']) > ftol:
        print('trim failed:\n' + str(res) + '\n')

    if debug_level == 2:
        print(res)
        print()
        for con in constraints:
            print('Constraint', con['type'], con['fun'](res['x'], *con['args']))
        print()

    if debug_level >= 1:
        print('Alpha (deg): '   , fdm['ic/alpha-deg'])
        print('Beta (deg): '    , fdm['ic/beta-deg'])
        print('Aileron: '       , fdm['fcs/aileron-cmd-norm'])
        print('Elevator: '      , fdm['fcs/elevator-cmd-norm'])
        print('Rudder:  '       , fdm['fcs/rudder-cmd-norm'])
        print('Flap:  '         , fdm['fcs/flap-cmd-norm'])
        print('Mixture:  '      , fdm['fcs/mixture-cmd-norm'])
        print('Throtle: '       , fdm['fcs/throttle-cmd-norm'])

    return ic


def trim_wings_level_flight(fdm, ic_h_sl_ft, ic_mach, ic_phi_rad, ic_psi_rad, ic_gamma_rad, debug_level=0):
    operation_point = trim_optimization(
        fdm=fdm,
        ic={
            'ic/h-sl-ft': ic_h_sl_ft,
            'ic/mach': ic_mach,
            'ic/phi-rad': ic_phi_rad,
            'ic/psi-true-rad': ic_psi_rad,
            'ic/gamma-rad': ic_gamma_rad,
        },
        design_vector=[
            'fcs/aileron-cmd-norm',
            'fcs/elevator-cmd-norm',
            'fcs/rudder-cmd-norm',
            'fcs/flap-cmd-norm',
            'fcs/mixture-cmd-norm',
            'fcs/throttle-cmd-norm[0]',
        ],
        method='SLSQP',
        eq_constraints= [
            lambda fdm: fdm['accelerations/udot-ft_sec2'],
            lambda fdm: fdm['accelerations/vdot-ft_sec2'],
            lambda fdm: fdm['accelerations/wdot-ft_sec2'],
            lambda fdm: fdm['accelerations/pdot-rad_sec2'],
            lambda fdm: fdm['accelerations/qdot-rad_sec2'],
            lambda fdm: fdm['accelerations/rdot-rad_sec2'],
        ],
        x0=[
            0,
            0,
            0,
            0,
            0.1,
            0.5
            ],
        debug_level = debug_level,
        bounds=[
            [-1, 1],
            [-1, 1],
            [-1, 1],
            [-1, 1],
            [0, 1],
            [0, 1]
            ],
    )
    return operation_point


if __name__ == '__main__':

    # Set jsbsim and flightgear
    aircraft_model='c172p'
    aircraft_path=(Path('.')).resolve()
    fdm = jsbsim.FGFDMExec(str(aircraft_path))
    #fdm.set_output_directive(str(aircraft_path/'fg_conn.xml'))
    fdm.set_debug_level(0)
    fdm.load_model(aircraft_model)

    # Initial Conditions
    # Position
    fdm['ic/lat-geod-rad'] = np.deg2rad(-23.42)     # Latitude (rad)
    fdm['ic/long-gc-rad'] = np.deg2rad(-46.47)      # Longitude (rad)
    fdm['ic/h-sl-ft'] =  500                       # ft

    # Attitude
    fdm['ic/phi-rad'] =   0                            # Roll (rad)
    fdm['ic/theta-rad'] = 0                            # Pitch (rad)   
    fdm['ic/psi-true-rad'] =   0                       # Yaw (rad)


    # Linear Velocities
    fdm['ic/mach'] = 0.25


    # Angular Velocities
    fdm['ic/p-rad_sec'] = 0                                    
    fdm['ic/q-rad_sec'] = 0                                     
    fdm['ic/r-rad_sec'] = 0

    fdm['forces/hold-down'] = 0                    
    fdm['fcs/throttle-cmd-norm'] = 1
    fdm['fcs/mixture-cmd-norm'] = 1
    fdm['propulsion/magneto_cmd'] = 3
    fdm['propulsion/starter_cmd'] = 1                                   

    fdm.run_ic()

    op_climb = trim_wings_level_flight(
                        fdm = fdm,
                        ic_h_sl_ft = 500,
                        ic_mach = 0.2,
                        ic_phi_rad = 0,
                        ic_psi_rad = 0,
                        ic_gamma_rad = np.deg2rad(5),
                        debug_level=2)