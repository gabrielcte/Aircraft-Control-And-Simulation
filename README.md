# Aircraft Control And Simulation

This repository contains scripts for:

* Steady-State Trim;
* Linearization;
* Nonlinear Simulation;
* Aircraft Control;

## Folders

| Pasta                    | Description                                                                              |
| ------------------------ | ---------------------------------------------------------------------------------------- |
| root                     | Contains scripts currently under development, evolving according to exploratory analyses |
| [aircraft](./aircraft)   | Contains aircraft models that will be used in Flight Gear                                |
| [engine](./engine)       | Contains engine models used by the aircraft                                              |
| [scripts](./scripts)     | Contains initialization scripts for Flight Gear                                          |
| [reference](./reference) | Contains manuals and documents used as reference for this work                           |


## Softwares

The Aircraft simulation, control and system analysis was conducted using three tools:

| Tool                                        | Application                                                                                       |
| ------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| [Flight Gear](https://www.flightgear.org/); | Graphical flight simulation software                                                              |
| [JSBSim](https://jsbsim.sourceforge.net/);  | Software that implements the flight dynamics model of aerial and spacial vehicles                 |
| [Python](https://www.python.org/);          | Multipurpose programming language. In this case, it's used to interact with FlightGear and JSBSim |

## Scripts

Bellow some description about the scripts:

| Script                    | Description                                                                                                                   |
| ------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| dynamic_simulation.ipynb  | Preliminary design that defines data for the spacecraft, reaction wheels, and mission.                                        |
| dynamic_simulation.py     | Simulation of the Cesna 172p aircraft mission with a non-linear model, where the equations of motion are integrated by JSBSim |

### Flight Gear Additional Settings

```
fgfs --fdm=null --native-fdm=socket,in,60,localhost,5550,udp --httpd=8080
```
