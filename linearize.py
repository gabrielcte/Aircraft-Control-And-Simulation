import numpy as np       
from ambiance import Atmosphere
from aerospace_ctrl_toolkit import (
    jsbsim_utils as jsbu,
    unit_conversion as uc
)


def machdot(fdm):
    
    U    = fdm['velocities/u-fps']
    Udot = fdm['accelerations/udot-ft_sec2']
    V    = fdm['velocities/v-fps']
    Vdot = fdm['accelerations/vdot-ft_sec2']
    W    = fdm['velocities/w-fps']
    Wdot = fdm['accelerations/wdot-ft_sec2']
    V_T  = fdm['velocities/vt-fps']

    atmosphere = Atmosphere(fdm['position/h-sl-meters'])
    speed_of_sound_meter_sec = atmosphere.speed_of_sound[0]
    speed_of_sound_ft_sec = uc.m2ft(speed_of_sound_meter_sec)

    machdot = ((U*Udot + V*Vdot + W*Wdot)/V_T)/speed_of_sound_ft_sec

    return machdot


def linearize_core(fdm, states, states_deriv, inputs, ic, dx=1e-4, n_round=3):

    assert len(states_deriv) == len(states)
    
    jsbu.set_ic0(fdm)
    
    n = len(states)
    p = len(inputs)

    def set_ic():

        for key in ic.keys():
            fdm[key] = ic[key]

        fdm.get_propulsion().init_running(0)
        fdm.run_ic()
    
    A = np.zeros((n, n))

    for i, state in enumerate(states):

        set_ic()

        start = {}
        start['custom/machdot'] = machdot(fdm)
        for item in fdm.get_property_catalog():
            property_name        = item.split(" ")[0]       # Remove qualquer flag (RW)
            start[property_name] = fdm.get_property_value(property_name)
        
        fdm[state] = start[state] + dx
        
        fdm.run_ic()

        fdm['custom/machdot'] = machdot(fdm)
        for j, state_deriv in enumerate(states_deriv):
            A[j, i] = (fdm[state_deriv] - start[state_deriv]) / dx

    B = np.zeros((n, p))

    for i, inp in enumerate(inputs):

        set_ic()

        start = {}
        start['custom/machdot']           = machdot(fdm)
        start['fcs/throttle-cmd-norm[0]'] = fdm['fcs/throttle-cmd-norm[0]']
        for item in fdm.get_property_catalog():
            property_name        = item.split(" ")[0]       # Remove qualquer flag (RW)
            start[property_name] = fdm.get_property_value(property_name)

        fdm[inp] = start[inp] + dx

        fdm.run_ic()

        fdm['custom/machdot'] = machdot(fdm)
        for j, state_deriv in enumerate(states_deriv):
            B[j, i] = (fdm[state_deriv] - start[state_deriv]) / dx
    
    del fdm
    A = np.round(A, n_round)
    B = np.round(B, n_round)

    return (A, B)


def linearize_longitudinal(fdm, operating_point, uw_format=False, h_augmentation=False):

    # States: U and W, or Vt and alpha
    if uw_format:
        states = ['ic/u-fps', 'ic/w-fps']
        states_deriv = ['accelerations/udot-ft_sec2', 'accelerations/wdot-ft_sec2']
    else:
        states = ['ic/mach', 'ic/alpha-rad']
        states_deriv = ['custom/machdot', 'aero/alphadot-rad_sec']

    # States: theta and Q
    states += ['ic/theta-rad', 'ic/q-rad_sec']
    states_deriv += ['velocities/thetadot-rad_sec', 'accelerations/qdot-rad_sec2']

    # State: h
    if h_augmentation:
        states += ['ic/h-agl-ft']
        states_deriv += ['velocities/h-dot-fps']

    inputs = ['fcs/throttle-cmd-norm[0]', 'fcs/elevator-cmd-norm']

    (A, B) = linearize_core(
        fdm          = fdm,
        states       = states,
        states_deriv = states_deriv,
        inputs       = inputs,
        ic           = operating_point,
    )
    return (A, B, states, inputs)


def linearize_lateral_directional(fdm, operating_point):
    states = ['ic/v-fps', 'ic/p-rad_sec', 'ic/r-rad_sec', 'ic/phi-rad', 'ic/psi-true-rad']
    inputs = ['fcs/aileron-cmd-norm', 'fcs/rudder-cmd-norm']
    (A, B) = linearize_core(
        fdm          = fdm,
        states       = states,
        states_deriv = ['accelerations/vdot-ft_sec2', 'accelerations/pdot-rad_sec2', 'accelerations/rdot-rad_sec2', 'velocities/phidot-rad_sec', 'velocities/psidot-rad_sec'],
        inputs       = inputs,
        ic           = operating_point,
    )
    return (A, B, states, inputs)


def short_period_aproximation(fdm, operating_point):
    # States: alpha and Q
    states = ['ic/alpha-rad', 'ic/q-rad_sec']
    states_deriv = ['aero/alphadot-rad_sec', 'accelerations/qdot-rad_sec2']
    inputs = ['fcs/elevator-cmd-norm']
    (A, B) = linearize_core(
        fdm          = fdm,
        states       = states,
        states_deriv = states_deriv,
        inputs       = inputs,
        ic           = operating_point,
    )
    
    return (A, B, states, inputs)


def dutch_roll_approximation(fdm, operating_point):
    # States: beta and R
    states = ['ic/beta-rad', 'ic/r-rad_sec' ]
    states_deriv = ['aero/betadot-rad_sec', 'accelerations/rdot-rad_sec2']
    inputs = ['fcs/aileron-cmd-norm', 'fcs/rudder-cmd-norm']
    (A, B) = linearize_core(
        fdm          = fdm,
        states       = states,
        states_deriv = states_deriv,
        inputs       = inputs,
        ic           = operating_point,
    )
    
    return (A, B, states, inputs)

