import jsbsim
import time
from pathlib                import Path
import numpy                as np
from enum                   import Enum
import matplotlib.pyplot    as plt
import matplotlib.image     as mpimg
import pandas               as pd

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
    flight_stage_no_control           = 0
    flight_stage_control              = 1


if __name__ == '__main__':
    
    # Simulação

    realtime     = False
    sim_period   = 3600
    num_steps = sim_period*100
    frame_time   = 0
    dt = sim_period/num_steps
    frame_period = dt
    flight_stage_current = FlightStages.flight_stage_no_control
    no_control_duration = 100
    control_duration = 600    

    # Set jsbsim and flightgear
    aircraft_model='cubesat'
    aircraft_path=(Path('.')).resolve()

    fdm = jsbsim.FGFDMExec(str(aircraft_path))
    #fdm.set_output_directive(str(aircraft_path/'fg_conn.xml'))
    fdm.set_debug_level(0)
    fdm.load_model(aircraft_model)
    fdm.set_dt(dt)                                            # Define o passo da simulação (s)

    # Initial Conditions
    # Position
    fdm['ic/lat-geod-rad'] = np.deg2rad(-82.365)     # Latitude (rad)
    fdm['ic/long-gc-rad'] = np.deg2rad(-41.076)      # Longitude (rad)
    fdm['ic/h-sl-ft'] = m2ft(569366.9993)            # ft

    # Attitude
    fdm['ic/phi-rad'] =   np.deg2rad(0)                            # Roll (rad)
    fdm['ic/theta-rad'] = np.deg2rad(0)                            # Pitch (rad)   
    fdm['ic/psi-true-rad'] =   np.deg2rad(0)                       # Yaw (rad)


     # Linear Velocities
    fdm['ic/u-fps'] = m2ft(-5.19923341417592e+003)
    fdm['ic/v-fps'] = m2ft(3.82519438208177e+003)
    fdm['ic/w-fps'] = m2ft(-3.97333292224794e+003)

    # Angular Velocities
    fdm['ic/p-rad_sec'] = np.deg2rad(0.01)                                    
    fdm['ic/q-rad_sec'] = np.deg2rad(0.02)                                     
    fdm['ic/r-rad_sec'] = np.deg2rad(-0.01)                                   

    fdm.run_ic()
     

        # Parâmetros do CubeSat
    h_CubeSat = 30e-002  # [m]
    l_CubeSat = 20e-002 # [m]
    c_CubeSat = 10e-002 # [m]
    m_CubeSat = 6 # [kg]
    J_1 = 1/12*m_CubeSat*((l_CubeSat**2)+(h_CubeSat**2)) # [kg.m²]
    J_2 = 1/12*m_CubeSat*((c_CubeSat**2)+(h_CubeSat**2)) # [kg.m²]
    J_3 = 1/12*m_CubeSat*((l_CubeSat**2)+(c_CubeSat**2)) # [kg.m²]
    J = np.diag([J_1, J_2, J_3]) # [kg.m²]

    # Parâmetros Roda de Reação
    V_RDR_Idle = 0.225 # [V]
    V_RDR_Operacao = 4.5 # [V]
    wRDR_max = 8000/9.5492965964254 # [rad/s]
    wRDR_Otima = 1000/9.5492965964254; # [rad/s]
    alpha = 1-wRDR_Otima/wRDR_max
    r = 1
    No = 3.84e-003 # [N.m]
    N_C = 7.06e-004 # [N.m]
    f = 1.21e-008*9.5492965964254 # [N.m/rad/s]
    m_RDR = 0.137 # [kg]
    r_RDR = 0.0435 # [m]
    J_axial_RDR = 0.5*m_RDR*r_RDR**2 # [kg*m²]

    # Declarando Variáveis
    T_res = np.zeros((3,num_steps), dtype='float32') # [N.m]
    N_app = np.zeros((3,num_steps), dtype='float32')  # [N.m]
    N_Friccao = np.zeros((3,num_steps), dtype='float32')  # [N.m]
    N_em = np.zeros((3,num_steps), dtype='float32')  # [N.m]
    aRDR = np.zeros((3,num_steps), dtype='float32')  # [rad/s²]
    wRDR = np.zeros((3,num_steps), dtype='float32')  # [rad/s]

    # Afinação do Controle de Atitude
    #ref_ang = np.array([fdm['ic/phi-rad'], fdm['ic/theta-rad'], fdm['ic/psi-true-rad']], dtype='float32') 
    ref_ang = np.array(np.deg2rad([10, 10, 30]), dtype='float32') 
    Se_angdt = np.array([0, 0, 0], dtype='float32') 
    e_ang = np.zeros((3,num_steps), dtype='float32')

    Kp_ang = 2
    Ki_ang = 0
    Kd_ang = 1

    # Afinação do Contole de Rotação da Roda de Reação
    Vapp = np.array([0, 0, 0], dtype='float32') 
    Xdc = np.array([0, 0, 0], dtype='float32') 
    ref_wrdr = np.array([0.0, 0.0, 0.0])
    Se_wrdrdt = np.array([0.0, 0.0, 0.0])
    e_wrdr = np.zeros((3,num_steps), dtype='float32') 
    Kp_wrdr = 0.6
    Ki_wrdr = 0.6*2*0.1
    Kd_wrdr = 0.6*0.125*0.1

    # Data frame
    data = []

    try:
           
        for i in range(num_steps):
        #while fdm.get_sim_time() <= sim_period:

            if flight_stage_current == FlightStages.flight_stage_no_control:


                if fdm.get_sim_time() > no_control_duration:
                    print('ACTIVATE CONTROL')
                    flight_stage_current = FlightStages.flight_stage_control
                    #break

            elif flight_stage_current == FlightStages.flight_stage_control:
                
                for j in range(3):                    
                    # Controle da Atitude do Satélite
                    theta_B_I_B = np.array([fdm['attitude/phi-rad'], fdm['attitude/theta-rad'], fdm['attitude/psi-rad']], dtype='float32') 
                    e_ang[j][i] = ref_ang[j]-theta_B_I_B[j]
                    

                    if (i>1):
                        de_ang = e_ang[j][i]-e_ang[j][i-1]

                    else:
                        de_ang = 0
                
                    Se_angdt[j] = Se_angdt[j]+e_ang[j][i]*dt
                    ref_wrdr[j] = Kp_ang*e_ang[j][i]+Ki_ang*Se_angdt[j]+Kd_ang*de_ang/dt # [V]
                    
                    # Controle de Rotação Roda de Reação 
                    e_wrdr[j][i] = ref_wrdr[j] - wRDR[j][i]
                                                            
                    if (i>1):
                        de_wrdr = e_wrdr[j][i]-e_wrdr[j][i-1]                        

                    else:
                        de_wrdr = 0

                    Se_wrdrdt[j] = Se_wrdrdt[j]+e_wrdr[j][i]*dt
                    Vapp[j] = Kp_wrdr*e_wrdr[j][i]+Ki_wrdr*Se_wrdrdt[j]+Kd_wrdr*de_wrdr/dt # [V]
                    
                    # Atuador
                    # Ciclo de Trabalho
                    if np.abs(Vapp[j])>V_RDR_Operacao:
                        Xdc[j] = 1

                    elif np.abs(Vapp[j])<V_RDR_Idle:
                        Xdc[j] = 0
                
                    elif np.abs(Vapp[j])>=V_RDR_Idle and np.abs(Vapp[j])<=V_RDR_Operacao:
                        coefs = np.polyfit([V_RDR_Idle, V_RDR_Operacao], [V_RDR_Idle/V_RDR_Operacao, 1], 1)
                        Xdc[j] = np.polyval(coefs, np.abs(Vapp[j]))
                
                    Xdc[j] = np.sign(Vapp[j])*Xdc[j]
                    
                    #Torques Roda de Reação
                    if np.abs(wRDR[j][i])>wRDR_max:
                        wRDR[j][i] = np.sign(wRDR[j][i])*wRDR_max # [rad/s]

                    if Xdc[j]>0:
                        r = 1-wRDR[j][i]/wRDR_max # [rad/s]

                    elif Xdc[j]<0:
                        r = 1+wRDR[j][i]/wRDR_max # [rad/s]

                    
                    N_Friccao[j][i+1] = N_C*np.sign(wRDR[j][i])+f*wRDR[j][i] # [N.m]
                    N_em[j][i+1] = Xdc[j]*2*No*alpha*r/(alpha**2+r**2) # [N.m]
                    N_app[j][i+1] = (N_em[j][i+1]-N_Friccao[j][i+1]) # [N.m]
                    aRDR[j][i+1] = N_app[j][i+1]/J_axial_RDR # [rad/s^2]                    
                    wRDR[j][i+1] = wRDR[j][i]+aRDR[j][i+1]*dt # [rad/s]
                    
                    # Aplicação do torque de controle na planta
                    fdm['actuator/RDR-x'] = Nm2Lbft(N_app[0][i+1])
                    fdm['actuator/RDR-y'] = Nm2Lbft(N_app[1][i+1])
                    fdm['actuator/RDR-z'] = -Nm2Lbft(N_app[2][i+1])

                if np.abs(wRDR[0][i]) == 761.11:
                    print('RDR eixo X Saturou')
                elif np.abs(wRDR[1][i]) == 761.11:
                    print('RDR eixo Y Saturou')
                elif np.abs(wRDR[2][i]) == 761.11:
                    print('RDR eixo Z Saturou')
                elif fdm.get_sim_time() > control_duration:
                    break                                            

                
            else:
                raise Exception('### ERROR: undefined flight stage!')
            
            new_data = {'sim-time-sec' : fdm.get_sim_time(),
                        'position/lat-geod-deg' : fdm['position/lat-geod-deg'],
                        'position/long-gc-deg' : fdm['position/long-gc-deg'],
                        'position/geod-alt-ft' : fdm['position/geod-alt-ft'],                    
                        'attitude/phi-rad' : fdm['attitude/phi-rad'],
                        'attitude/theta-rad' : fdm['attitude/theta-rad'],   
                        'attitude/psi-rad' : fdm['attitude/psi-rad'],
                        'velocities/p-rad_sec' : fdm['velocities/p-rad_sec'],
                        'velocities/q-rad_sec': fdm['velocities/q-rad_sec'],
                        'velocities/r-rad_sec': fdm['velocities/q-rad_sec'],
                        'velocities/phidot-rad_sec' : fdm['velocities/phidot-rad_sec'],                                
                        'velocities/thetadot-rad_sec' : fdm['velocities/thetadot-rad_sec'],                                     
                        'velocities/psidot-rad_sec' : fdm['velocities/psidot-rad_sec'],
                        }
            
            data.append(new_data)       

            print(f"Time: {fdm.get_sim_time():.2f} s\
                    Atuator X: {wRDR[0][i]:.2f} rad/sec\
                    Atuator Y: {wRDR[1][i]:.2f} rad/sec\
                    Atuator Z: {wRDR[2][i]:.2f} rad/sec", end='\r', flush=True)
            
            fdm.run()

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
                        'velocities/p-rad_sec',
                        'velocities/q-rad_sec',
                        'velocities/r-rad_sec',
                        'velocities/phidot-rad_sec',                                
                        'velocities/thetadot-rad_sec',                                     
                        'velocities/psidot-rad_sec',
                        'control/erro_phi',
                        'control/erro_theta',
                        'control/erro_psi',     
                ])   

        plt.figure(1)
        plt.title('Ground Track')
        plt.plot(df['position/long-gc-deg'], df['position/lat-geod-deg'],':b')

        # Ler a imagem

        contour = mpimg.imread('mapaContorno.jpg')
        # Configurar a exibição da imagem

        plt.imshow(contour, extent=[-180, 180, -90, 90], cmap='gray')

        # Configurar os rótulos dos eixos
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")

        # Definir limites dos eixos
        plt.xlim([-180, 180])
        plt.ylim([-90, 90])

        # Definir as marcações dos eixos x
        plt.xticks([-180, -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150, 180],
                ['180W', '150W', '120W', '90W', '60W', '30W', '0', '30E', '60E', '90E', '120E', '150E', '180E'])

        # Definir as marcações dos eixos y
        plt.yticks([90, 75, 60, 45, 30, 15, 0, -15, -30, -45, -60, -75, -90],
                ['90N', '75N', '60N', '45N', '30N', '15N', '0', '15S', '30S', '45S', '60S', '75S', '90S'])

        # Configurar a grade e proporções iguais
        plt.axis('on')
        plt.axis('equal')
        plt.grid(True)


        plt.figure(2)
        # Plotar as curvas de atitude
        plt.plot(df['sim-time-sec'], np.rad2deg(df['attitude/phi-rad']), ':b', label=r'$\phi$')
        plt.plot(df['sim-time-sec'], np.rad2deg(df['attitude/theta-rad']), ':r', label=r'$\theta$')
        plt.plot(df['sim-time-sec'], np.rad2deg(df['attitude/psi-rad']), ':g', label=r'$\psi$')
        plt.legend()
        plt.xlabel('Time [sec]')
        plt.ylabel('Attitude [Deg]')
        plt.title('Evolução da Atitude do Cubesat 6U')
        plt.grid()

        plt.figure(3)
        plt.plot(df['sim-time-sec'], np.rad2deg(df['velocities/phidot-rad_sec']),':b', label=r'$\dot{\phi}$')
        plt.plot(df['sim-time-sec'], np.rad2deg(df['velocities/thetadot-rad_sec']),':r', label=r'$\dot{\theta}$')
        plt.plot(df['sim-time-sec'], np.rad2deg(df['velocities/psidot-rad_sec']),':g', label=r'$\dot{\psi}$')
        plt.legend()
        plt.xlabel('Time [sec]')
        plt.ylabel('Velocidade Angular [Deg/sec]')
        plt.title('Velocidade Angular do Cubesat 6U')
        plt.grid()



        # Mostrar a imagem
        plt.show()

        # Salva data frame como csv
        df.to_csv('cubesat_dataframe.csv', index=False)
