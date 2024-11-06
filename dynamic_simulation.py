import jsbsim
import time
from pathlib                import Path
import numpy                as np
from enum                   import Enum
import pandas               as pd
import trim

def ft2m(feet_value):
    meter_value = feet_value*0.3048
    return meter_value

def m2ft(meter_value):
    feet_value = meter_value/0.3048
    return feet_value

def Nm2Lbft(Nm):
    Lbft = Nm*0.7375621493
    return Lbft

class FlightStages(Enum):
    flight_stage_0           = 0
    flight_stage_1           = 1
    flight_stage_2           = 2   


if __name__ == '__main__':
    
    # Simulação

    realtime     = True
    sim_period   = 3600
    num_steps = sim_period*100
    frame_time   = 0
    dt = sim_period/num_steps
    frame_period = dt
    flight_stage_current = FlightStages.flight_stage_0

    # Set jsbsim and flightgear
    aircraft_model='c172p'
    aircraft_path=(Path('.')).resolve()

    fdm = jsbsim.FGFDMExec(str(aircraft_path))
    fdm.set_output_directive(str(aircraft_path/'fg_conn.xml'))
    fdm.set_debug_level(0)
    fdm.load_model(aircraft_model)
    fdm.set_dt(dt)                                            # Define o passo da simulação (s)

    # Initial Conditions
    # Position
    fdm['ic/lat-geod-rad'] = np.deg2rad(-23.432)     # Latitude (rad)
    fdm['ic/long-gc-rad'] = np.deg2rad(-46.470)      # Longitude (rad)
    fdm['ic/h-sl-ft'] = 2458                  # ft
    fdm['ic/terrain-altitude-ft'] = 2458 
    fdm['ic/h-agl-ft'] = m2ft(10)                    # ft

    # Attitude
    fdm['ic/phi-rad'] =   np.deg2rad(0)                            # Roll (rad)
    fdm['ic/theta-rad'] = np.deg2rad(0)                            # Pitch (rad)   
    fdm['ic/psi-true-rad'] =   np.deg2rad(73.7)                       # Yaw (rad)


    # Linear Velocities
    fdm['ic/u-fps'] = m2ft(0)
    fdm['ic/v-fps'] = m2ft(0)
    fdm['ic/w-fps'] = m2ft(0)

    # Angular Velocities
    fdm['ic/p-rad_sec'] = np.deg2rad(0)                                    
    fdm['ic/q-rad_sec'] = np.deg2rad(0)                                     
    fdm['ic/r-rad_sec'] = np.deg2rad(0)                                   

    fdm.run_ic()

    # Data frame
    data = []
    
    #
    stage_0_duration = 3
    V_take_off = m2ft(35) 

    try:
           
        for i in range(num_steps):

            if flight_stage_current == FlightStages.flight_stage_0:
                fdm['forces/hold-down'] = 1
                fdm['propulsion/tank[0]/contents-lbs'] = 185
                fdm['propulsion/tank[1]/contents-lbs'] = 185

                # Configura vento
                ## Cross Wind
                #fdm['atmosphere/crosswind-fps'] = 42.195246427529995 # Equivalente a 25 Knots

                ## Head wind
                #fdm['atmosphere/headwind-fps'] = 42.195246427529995 # Equivalente a 25 Knots 

                ## Gust Wind
                #fdm['atmosphere/gust-north-fps'] = 42.195246427529995 # Equivalente a 25 Knots
                #fdm['atmosphere/gust-east-fps'] = 42.195246427529995 # Equivalente a 25 Knots
                #fdm['atmosphere/gust-down-fps'] = 42.195246427529995 # Equivalente a 25 Knots

                ## Turbulence
                #fdm['atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps'] = 1 
                #fdm['atmosphere/turbulence/milspec/severity'] = 1
                #fdm['atmosphere/turb-gain'] = 0
                #fdm['atmosphere/turb-rate'] = 0
                #fdm['atmosphere/turb-rhythmicity'] = 0


                if fdm.get_sim_time() > stage_0_duration:
                    print('Starting')
                    flight_stage_current = FlightStages.flight_stage_1
                    #break

            elif flight_stage_current == FlightStages.flight_stage_1:
                    fdm['forces/hold-down'] = 0                    
                    fdm['fcs/throttle-cmd-norm'] = 1
                    fdm['fcs/mixture-cmd-norm'] = 1
                    fdm['propulsion/magneto_cmd'] = 3
                    fdm['propulsion/starter_cmd'] = 1
                    
                    if fdm['velocities/vt-fps'] > V_take_off and fdm['position/h-agl-ft'] > m2ft(10):
                        print('Starting')
                        flight_stage_current = FlightStages.flight_stage_2
                        #break
            elif flight_stage_current == FlightStages.flight_stage_2:
                    op_climb = trim.trim_wings_level_flight(
                        fdm = fdm,
                        ic_h_sl_ft = fdm['position/h-sl-ft'],
                        ic_mach = fdm['velocities/mach'],
                        ic_phi_rad = fdm['attitude/phi-rad'],
                        ic_psi_rad = fdm['attitude/psi-rad'],
                        ic_gamma_rad = np.deg2rad(10),
                        debug_level=0)
                    break

                    
                    if fdm['position/h-agl-ft'] > 500:
                        print('Starting')
                        flight_stage_current = FlightStages.flight_stage_2
                        break
                
                
            else:
                raise Exception('### ERROR: undefined flight stage!')
            
            new_data = {
                        'sim-time-sec' : fdm.get_sim_time(),
                        'position/lat-geod-deg' : fdm['position/lat-geod-deg'],
                        'position/long-gc-deg' : fdm['position/long-gc-deg'],
                        'position/geod-alt-ft' : fdm['position/geod-alt-ft'],
                        'position/h-agl-ft' : fdm['position/h-agl-ft'],                                       
                        'attitude/phi-rad' : fdm['attitude/phi-rad'],
                        'attitude/theta-rad' : fdm['attitude/theta-rad'],   
                        'attitude/psi-rad' : fdm['attitude/psi-rad'],
                        'aero/alpha-deg' : fdm['aero/alpha-deg'],
                        'aero/beta-deg' : fdm['aero/beta-deg'],
                        'velocities/u-fps' : fdm['velocities/u-fps'],
                        'velocities/v-fps' : fdm['velocities/v-fps'],
                        'velocities/w-fps' : fdm['velocities/w-fps'],
                        'velocities/p-rad_sec' : fdm['velocities/p-rad_sec'],
                        'velocities/q-rad_sec' : fdm['velocities/q-rad_sec'],
                        'velocities/r-rad_sec' : fdm['velocities/r-rad_sec'],
                        'velocities/phidot-rad_sec' : fdm['velocities/phidot-rad_sec'],                                
                        'velocities/thetadot-rad_sec' : fdm['velocities/thetadot-rad_sec'],                                     
                        'velocities/psidot-rad_sec' : fdm['velocities/psidot-rad_sec'],
                        }
            
            data.append(new_data)       

            print(f"Time: {fdm.get_sim_time():.2f} s\
                    Velocidade U: {ft2m(fdm['velocities/u-fps']):.2f} m/sec\
                    Altitude: {ft2m(fdm['position/h-agl-ft']):.2f} m\
                    Alpha: {fdm['aero/alpha-deg' ]:.2f} deg", end='\r', flush=True)
            
            fdm.run()
            
            if fdm['position/h-agl-ft'] < 0:
                break                


            if realtime:
                
                if fdm.get_sim_time() > frame_time:
                    frame_time += frame_period
                    time.sleep(frame_period)

    except ValueError as ve:
        print(f"Erro de valor encontrado: {ve}")

    except KeyError as ke:
        print(f"Chave não encontrada no dicionário: {ke}")

    except FileNotFoundError as fe:
        print(f"Arquivo não encontrado: {fe}")

    except Exception as e:
        print(f"A simulação encontrou um erro: {type(e).__name__}: {e}")

    finally:
        print('END')
                   
        df = pd.DataFrame(data, columns=[
                        'sim-time-sec',
                        'position/lat-geod-deg',
                        'position/long-gc-deg',
                        'position/geod-alt-ft',                    
                        'attitude/phi-rad',
                        'attitude/theta-rad',   
                        'attitude/psi-rad',
                        'aero/alpha-deg',
                        'aero/beta-deg',
                        'velocities/u-fps',
                        'velocities/u-fps',
                        'velocities/u-fps',
                        'velocities/p-rad_sec',
                        'velocities/q-rad_sec',
                        'velocities/r-rad_sec',
                        'velocities/phidot-rad_sec',                               
                        'velocities/thetadot-rad_sec',                                    
                        'velocities/psidot-rad_sec',
                ])   
