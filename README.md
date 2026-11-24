# Quadrotor Reinforcement Learning in Flightmare

**PPO-based quadrotor flight control with collective-thrust and body-rate actions, and policy export for evaluation in PX4–Gazebo.**

This repository is the **Flightmare training component** of the [Robust Goal-Directed Aerial Manipulation project](https://github.com/1shit4/aerial-manipulation-rl-control). The broader work investigates learning coordinated flight commands and combining them with lower-level controllers for robust aerial manipulation. Here, the focus is on **training a standalone quadrotor policy**, defining its observation/action interface, and transferring the learned policy to a separate flight-control simulation.

The implementation builds on the open-source [Flightmare simulator](https://github.com/uzh-rpg/flightmare), with modifications to the quadrotor environment, dynamics, rewards, training scripts, and policy-export workflow.

## Flight-Control Policy

The primary training configuration uses **Proximal Policy Optimization (PPO2)** with an MLP actor–critic policy. Rather than directly predicting individual rotor speeds, the policy generates **collective thrust and desired body rates (CTBR)**, allowing the simulated quadrotor's inner rate controller and motor allocator to calculate rotor thrusts.
The policy receives a 19-dimensional observation of the quadrotor state and outputs four control commands: collective thrust and three desired body rates. A low-level angular-rate controller and motor-allocation model convert these commands into individual rotor thrusts for simulation.

### Observation space

The [quadrotor environment](flightlib/src/envs/quadrotor_env/quadrotor_env.cpp) constructs a **19-dimensional observation**:

| Observation | Dimensions | Representation |
| --- | ---: | --- |
| Vehicle position | 3 | Position divided by 10 |
| Attitude | 9 | Flattened rotation matrix |
| Linear velocity | 3 | Velocity divided by 10 |
| Angular velocity | 3 | Body angular rates |
| Accumulated yaw term | 1 | Integrated yaw-angle term |
| **Total** | **19** | |

The policy's four outputs are converted by the environment into the following commands:

| Action | Conversion in the environment |
| --- | --- |
| Collective thrust/acceleration | `action[0] + 9.81` |
| Desired roll rate | `5 × action[1]` |
| Desired pitch rate | `5 × action[2]` |
| Desired yaw rate | `0.01 × action[3]` |

The resulting CTBR reference passes through the rate-feedback and motor-allocation model in [`quadrotor.cpp`](flightlib/src/objects/quadrotor.cpp). The simulator also models motor response dynamics.

## Training Objective and Reward Design

The configured training task stabilizes the quadrotor around a **fixed reference position of `(0, 0, 10 m)`**, with randomized initial positions, velocities, and orientations. This provides a hovering/position-stabilization policy that can subsequently be evaluated through a separate simulator and control interface.

The reward combines:

- **Position tracking:** an exponential penalty based on distance from the reference position.
- **Attitude stabilization:** an exponential penalty based on orientation error.
- **Motion penalties:** quadratic linear- and angular-velocity errors.
- **Control regularization:** penalties associated with actions, body-rate commands, and accumulated yaw.
- **Survival:** a small per-step reward, with termination on ground contact.

Relevant parameters are in [`quadrotor_env.yaml`](flightlib/configs/quadrotor_env.yaml), with the reward computation in [`quadrotor_env.cpp`](flightlib/src/envs/quadrotor_env/quadrotor_env.cpp).

### Randomized dynamics and initial conditions

The environment randomizes the initial vehicle state at reset and contains variations in vehicle **mass, arm length, and rotational inertia**. The [dynamics implementation](flightlib/src/objects/quadrotor.cpp) also includes separate motor spin-up and spin-down time constants. These experiments examine policy behavior under changes to initial state and vehicle dynamics rather than only a single nominal configuration.

## PPO Training and Model Export

The main entry point is [`run_drone_control.py`](flightrl/examples/run_drone_control.py), using the existing Flightmare vectorized environment and the **TensorFlow-based `rpg_baselines` PPO2 implementation**.

The checked-in configuration specifies:

| Setting | Value |
| --- | --- |
| Parallel environments | 100 |
| Environment threads | 10 |
| Simulation step | 0.02 s |
| Episode time limit | 5 s |
| Actor and critic | Two 128-unit hidden layers each (ReLU) |
| Discount factor | 0.99 |
| GAE parameter | 0.95 |
| PPO clip range | 0.2 |
| Learning rate | `3e-4` |

The training script saves PPO checkpoints and includes a **TensorFlow-to-ONNX export path** for inference outside Flightmare. Additional scripts examine exported ONNX outputs and conversions of learned policy parameters.

Once Flightmare and its Python dependencies have been installed and `FLIGHTMARE_PATH` is configured, the training script can be invoked from `flightrl/examples`:

```bash
python3 run_drone_control.py --train 1 --render 0
```

To export an existing PPO checkpoint to ONNX:

```bash
python3 run_drone_control.py --train 0 --weight /path/to/checkpoint.zip
```

The export branch writes an ONNX model to the working directory. A compatible Flightmare build and the script's older TensorFlow/Stable Baselines dependencies are required; model checkpoints are not supplied in this source-only distribution.

### Additional control-action experiments

[`drone_controlctbr_thrust.py`](flightrl/examples/drone_controlctbr_thrust.py) contains an **alternative action-allocation wrapper** that maps body-torque and collective-thrust commands to individual rotor thrusts. It is a separate control-interface experiment, rather than the CTBR action mapping used by the main training entry point.

## Evaluation in PX4–Gazebo

The exported Flightmare policy was evaluated in a separate **PX4–Gazebo** simulation to examine how learned flight commands behave when the vehicle dynamics, observation interfaces, and actuation pipeline differ from those used during training.

The companion repository, **[RLQuadcopterControlFlightmare](https://github.com/1shit4/RLQuadcopterControlFlightmare)**, implements the ROS 2/PX4 observation reconstruction, ONNX inference, body-rate feedback, and conversion to individual motor commands. That repository also contains the sim-to-sim flight-control experiments; this repository contains the **training-side implementation**.

## Key Files

| Path | Purpose |
| --- | --- |
| [`flightlib/src/envs/quadrotor_env/quadrotor_env.cpp`](flightlib/src/envs/quadrotor_env/quadrotor_env.cpp) | Observation and action definitions, reward computation, resets, and simulation stepping |
| [`flightlib/src/objects/quadrotor.cpp`](flightlib/src/objects/quadrotor.cpp) | Rate feedback, motor allocation, motor dynamics, and vehicle-parameter randomization |
| [`flightlib/configs/quadrotor_env.yaml`](flightlib/configs/quadrotor_env.yaml) | Quadrotor dynamics and reward parameters |
| [`flightlib/configs/vec_env.yaml`](flightlib/configs/vec_env.yaml) | Parallel-environment configuration |
| [`flightrl/examples/run_drone_control.py`](flightrl/examples/run_drone_control.py) | PPO training, checkpoint handling, and ONNX export |
| [`flightrl/examples/drone_controlctbr_thrust.py`](flightrl/examples/drone_controlctbr_thrust.py) | Alternative action-allocation experiment |
| [`flightrl/examples/test_onnx.py`](flightrl/examples/test_onnx.py) | ONNX inference and observation/action inspection |

## Related Projects and Attribution

- [Robust Goal-Directed Aerial Manipulation](https://github.com/1shit4/aerial-manipulation-rl-control) — overall research project and repository index.
- [Flightmare Policy Inference in PX4–Gazebo](https://github.com/1shit4/RLQuadcopterControlFlightmare) — evaluation-side implementation for exported policies.
- [Original Flightmare](https://github.com/uzh-rpg/flightmare) — quadrotor simulator developed by the Robotics and Perception Group at the University of Zurich. The original Flightmare framework and baseline code belong to their authors; this repository documents research modifications and experiments built on that framework.

*Research conducted at the Robotics Research Center, IIIT Hyderabad.*
