# UAV navigation in unknown complex environment

## Survey

- [A survey on vision-based UAV navigation](https://www.tandfonline.com/doi/full/10.1080/10095020.2017.1420509), 2018
- [Insights on obstacle avoidance for small unmanned aerial systems from a study of flying animal behavior](https://www.sciencedirect.com/science/article/abs/pii/S0921889017301197), 2018
- [A Review of Autonomous Obstacle Avoidance Technology for Multi-rotor UAVs](https://ieeexplore.ieee.org/abstract/document/8812473?casa_token=AXcXHro-5y8AAAAA:Vhan9sNCaGMHiHi61rf_OZG8HKyieTvpobpJXgJmVn07HYERdvji2eG2dghrD3jSrH48HKpSfSs), 2018
- [A literature review of UAV 3D path planning](https://ieeexplore.ieee.org/abstract/document/7053093), 2014
- [Survey on computational-intelligence-based UAV path planning](https://www.sciencedirect.com/science/article/abs/pii/S0950705118302636), 2018
- [A Survey of Motion Planning Algorithms from the Perspective of Autonomous UAV Guidance](https://link.springer.com/article/10.1007/s10846-009-9383-1), 2010
- [Survey of Motion Planning Literature in the Presence of Uncertainty: Considerations for UAV Guidance](https://link.springer.com/article/10.1007/s10846-011-9642-9), 2012

## Traditional method

### Multirotor

### Fixed-wing

### Flapping-wing

## Learning-based method

- Survey
  - [Drone deep reinforcement learning: A review](https://www.mdpi.com/2079-9292/10/9/999), 2021
    - targeted the guidance, navigation, and control (GNC) of UAVs.
    - RL for UAV path planning
  - [Deep reinforcement learning based mobile robot navigation: A review](https://ieeexplore.ieee.org/document/9409758), 2021
    - Focus on mobile robot
  - [A review of deep learning methods and applications for unmanned aerial vehicles](https://www.hindawi.com/journals/js/2017/3296874/), 2017
    - Focus on DL
    - Supervised and unsupervised learning
  - [Deep Learning and Reinforcement Learning for Autonomous Unmanned Aerial Systems: Roadmap for Theory to Deployment](https://arxiv.org/abs/2009.03349), 2020
    - good paper
      - overview of DL and RL
      - How DL used on UAS
      - How RL used on UAS
      - directions to choose appropriate simulation suites and hardware platforms
      - open problems and challenges
- Multirotor
  - [Reinforcement Learning for Autonomous UAV Navigation Using Function Approximation](https://ieeexplore.ieee.org/document/8468611), 2018
    - Quadrotor，NAV，Sim and Real、RL（Q-Learning）
  - [Path Planning for UAV Ground Target Tracking via Deep Reinforcement Learning](https://ieeexplore.ieee.org/abstract/document/8984371), 2020
    - UAV target tracking and obstacle avoidance
    - Fixed-wing, range finder，**not MAV**
    - Simulation
    - DDPG
  - [Memory-based Deep Reinforcement Learning for Obstacle Avoidance in UAV with Limited Environment Knowledge](https://ieeexplore.ieee.org/abstract/document/8917687), 2018
  - Quadrotor，OA，DRQN+Attention
    - 特色：RNN+Temporal Attention
    - conditional generative adversarial network for depth estimation
    - 60 Hz on NVIDIA GeForce GTX 1050 mobile GPU
  - [Autonomous UAV Navigation: A DDPG-Based Deep Reinforcement Learning Approach](https://ieeexplore.ieee.org/abstract/document/9181245), 2020
    - DDPG,3D env, NAV
  - [Obstacle Avoidance Drone by Deep Reinforcement Learning and Its Racing with Human Pilot](https://www.mdpi.com/2076-3417/9/24/5571), 2019
    - Quadrotor
    - using multi perception: RGB, Depth and RGB+Depth
    - using multi algorithms: DQN，Actor-Critic RL，TRPO, PPO
    - utilize diverse RL within two categories: (1) discrete action space and (2) continuous action space.
    - Results suggest that our best continuous algorithm easily outperformed the discrete ones and yet was similar to an expert pilot
  - [Autonomous navigation of UAV by using real-time model-based reinforcement learning](https://ieeexplore.ieee.org/abstract/document/7838739/), 2016
    - model-based RL TEXPLORE
    - find an efficient route when it is constrained in battery life
    - simulated quadcopter UAV in ROS and Gazebo environment
    - our approach significantly outperforms Q-learning based method
  - [UAV navigation in high dynamic environments: A deep reinforcement learning approach](https://www.sciencedirect.com/science/article/pii/S1000936120302247), 2020
    - high dynamic environments, LSTM
    - a clipped DRL loss function is proposed
    - propose a distributed DRL framework containing two sub-networks, namely the Avoid Network and the Acquire Network
    - 2D navigation, using range finders
  - [NavREn-Rl: Learning to fly in real environment via end-to-end deep reinforcement learning using monocular images](https://ieeexplore.ieee.org/abstract/document/8600838/), 2019
    - DDQN
    - small number of expert data and knowledge based data aggregation is integrated into the RL process to aid convergence
    - Parrot AR drone real test
  - [A Vision Based Deep Reinforcement Learning Algorithm for UAV Obstacle Avoidance](https://arxiv.org/abs/2103.06403), 2021
  - [Deep-Reinforcement-Learning-Based Autonomous UAV Navigation with Sparse Rewards](), 2020
    - adopt the sparse reward scheme
    - using prior policy (nonex- pert helper) that might be of poor performance is available to the learning agent.
  - [Collision-free UAV navigation with a monocular camera using deep reinforcement learning](), 2020
    - monocular camera
    - OBJECT DETECTION-ASSISTED DRL OD+DQN
    - proposed framework reduces flying times to- wards given destinations by 25%, and cuts down 50% of unnecessary turns.
  - [Autonomous Navigation of UAVs in Large-Scale Complex Environments: A Deep Reinforcement Learning Approach](), 2019
    - POMDP, LSTM
    - Fast-RDPG
    - 3D
  - [Generalization through Simulation: Integrating Simulated and Real Data into Deep Reinforcement Learning for Vision-Based Autonomous Flight](), 2019
    - Focus on generalization from sim to real
    - Evaluated with a nano aerial vehicle (NAV), Crazyflie 2.0
    - First, we train a deep neural network Q-function using deep reinforcement learning in a visually diverse set of **simulated environments**.
    - Then, we create the deep neural network action-conditioned reward prediction model, in which we use the perception layers from the simulation-trained Q- function to process the input image state.
    - Next, we train the action-conditioned reward prediction model using **real-world data** gathered by the robot; however, when training the model, we do not update the parameters of the perception layers.
  - [A Deep Reinforcement Learning Framework for UAV Navigation in Indoor Environments](), 2019
    - Formulate the UAV navigation problem using MDP and POMDP
    - separating the search problem into high-level planning and low-level action under uncertainty
  - [Uncertainty-Aware Reinforcement Learning for Collision Avoidance](), 2017
    - uncertainty-aware model-based learning algorithm
    - 16 by 16 grayscale image
  - [Deep reinforcement learning for drone navigation using sensor data](), 2021
    - PPO
    - incremental curriculum learning
    - LSTM
  - [Deep Reinforcement Learning-based UAV Navigation and Control: A Soft Actor-Critic with Hindsight Experience Replay Approach](), 2021
    - Focused on algorithm, propose SACHER (soft actor-critic(SAC) with hindsight experience replay (HER))
    - Out perform SAC and DDPG
  - [Deep Reinforcement Learning for End-to-End Local Motion Planning of Autonomous Aerial Robots in Unknown Outdoor Environments: Real-Time Flight Experiments](), 2021
    - Using laser range finder to autonomously navigate among obstacles and achieve a user-specified goal location in a GPS-denied environment
    - planning smooth forward linear velocity and heading rates
  - [A Deep Reinforcement Learning Method with Action Switching for Autonomous Navigation](https://ieeexplore.ieee.org/document/9549631/), 2021
    - PPO with Action Switching
    - PID is used as baseline controller
  - [Vision Based Drone Obstacle Avoidance by Deep Reinforcement Learning](), 2021
    - Use the depth maps as input and combine SAC with a variational auto-encoder (VAE), compared with TD3
    - The output of the actor network has two values ranging from −1 to 1, representing the velocity of the drone in the y direction (left and right directions) and z direction (up and down directions)
  - [Learning to Fly with Deep Reinforcement Learning](), 2021
    - PPO-Clip
    - Navigation controller without obstacle avoidance
  
- Fixed-wing
  - [End-to-End Deep Reinforcement Learning for Image-Based UAV Autonomous Control](), 2021
    - Fixed wing vision-based landing
    - map the input image directly to the continuous actuator control command
    - a comparison with a nonlinear model predictive (NMPC) technique has been proposed in simulation
  - [UAV Path Planning Based on Multi-Layer Reinforcement Learning Technique](), 2021
    - multi-layer path planning algorithm
      - higher layer deals with the local information (short-term strategy)
      - lower layer deals with the global information (long-term strategy)
      - B-spline curve approach is applied for on-line path smoothing
- Flapping-wing
- Simulator
  - [AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles](https://link.springer.com/chapter/10.1007/978-3-319-67361-5_40), 2017
    - built on Unreal Engine that offers physically and visually realistic simulations for both of these goals
  - [Air Learning: An AI Research Platform for Algorithm-Hardware Benchmarking of Autonomous Aerial Robots](), 2019
    - an AI research platform for benchmarking algorithm-hardware performance and energy efficiency trade-offs.
    - quality-of-flight (QoF) metrics
    - AirSim for sim and crazyflie for real
  - [RotorS---A Modular Gazebo MAV Simulator Framework](https://github.com/ethz-asl/rotors_simulator), 2016
  - [FlightGoogles](https://flightgoggles.mit.edu/), 2019
  - [Flightmare: A Flexible Quadrotor Simulator](https://uzh-rpg.github.io/flightmare/), 2021
  - [A Survey of UAV Simulation With Reinforcement Learning](https://www.chenshiyu.top/blog/2020/05/25/A-Survey-of-UAV-Simulation-With-Reinforcement-Learning/), blog
