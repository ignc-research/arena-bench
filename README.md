![](http://img.shields.io/badge/stability-stable-orange.svg?style=flat)
[![Linux](https://svgshare.com/i/Zhy.svg)](https://svgshare.com/i/Zhy.svg)
[![made-with-python](https://img.shields.io/badge/Made%20with-Python-1f425f.svg)](https://www.python.org/)

# arena-bench
This repository provides the code used in our paper "Arena-Bench: A Benchmarking Suite for Obstacle Avoidance Approaches in Highly Dynamic Environments" (preprint coming soon). The scope of this project is to benchmark dynamic obstacle avoidance approaches on different robotic systems in highly dynamic simulation environments.

- [Benchmark scenarios](#benchmark-scenarios)
- [Benchmark architecture](#benchmark-architecture)
- [Running the Benchmark](#running-the-benchmark)
    - [Prerequisites](#prerequisites)
    - [Installation](#installation)
    - [Manually running the benchmark](#manually-running-the-benchmark)
- [Thanks](#thanks)
---
## Benchmark scenarios
Benchmarking scenarios were developed uniquely combining the following components
| **Robot** | **World**  | **Planner**  | **Obstacles** |
| :-------- | :------------ | :---------- | :------- |
| - `jackal`  <br> - `burger` <br> - `robotino` (rto) | - `small_warehouse` <br> - `map2` <br> - `map5` | - `teb` <br> - `dwa` <br> - `mpc` <br> - `rosnav` <br> - `cadrl` <br>  | - `5` <br> - `10` |

The following clips show examples of the implemented scenarios:


 <img height="320" src="/docs/imgs/map5-jackal.gif">   <img height="320" src="/docs/imgs/sw-burger.gif">


---
## Benchmark architecture
<img src="/docs/imgs/architecture.jpg">
The repository is structured into a modular architecture with four models at its core.

- [Arena-Tools](https://github.com/ignc-research/arena-tools):
this module is being responsible to create custom worlds, scenarios, and obstacles and adding those to the respective location. It is possible to use arena-bench without arena-tools as several worlds, scenarios and obstacles have already been included, including all scenarios and worlds to reproduce this paper.

- [Arena-Rosnav](https://github.com/ignc-research/arena-rosnav):
this module contains a custom task-generator and the 2D simulation engine Flatland, enabling robot simulation in 2D Flatland environments (note that this repository has been added recently to the arena-bench repository, and is therefore not part of the original paper).

- [Arena-Rosnav-3D](https://github.com/ignc-research/arena-rosnav-3D):
this module contains a custom task-generator and the 3D simulation engine Gazebo, enabling robot simulation in 3D Gazebo environments.

- [Arena-Evaluation](https://github.com/ignc-research/arena-evaluation):
this module can be used to record and evaluate simulation runs and visualize Robot performance by creating qualitative and quantitative plots.

> **NOTE**: All modular components have their individual repository and are currently being extended with additional features

---
## Running the Benchmark
#### Prerequisites
Below is the software we used. We cannot guarantee older versions of the software to work. Yet, newer software is most likely working just fine.

| Software      | Version        |
| ------------- | -------------- |
| OS            | Ubuntu 20.04.4 |
| Python        | 3.8.10         |

#### Installation
To install the repo run:
```bash
wget https://raw.githubusercontent.com/ignc-research/arena-bench/main/setup.sh -O - | bash
```
#### Manually running the benchmark
You can run specific scenarios, be using the following syntax:
<pre class="devsite-click-to-copy">
roslaunch arena_bringup start_arena_gazebo.launch local_planner:=<var>PLANNER</var> world:=<var>WORLD</var> model:=<var>ROBOT</var> scenario_file:=<var>SCENARIO_FILE</var>
</pre>
with the choice of:
- `PLANNER`: dwa, teb, mpc, rosnav, cadrl
- `WORLD`: small_warehouse, map2, map5
- `ROBOT`: turtlebot3_burger, jackal, rto
- `SCENARIO_FILE`: map2_obs05.json, map2_obs10.json, map5_obs05.json, map5_obs10.json, small_warehouse_obs05.json, small_warehouse_obs10.json

Example command:
```bash
workon rosnav
roslaunch arena_bringup start_arena_gazebo.launch local_planner:=dwa world:=map2 model:=turtlebot3_burger scenario_file:=map2_obs05.json
```
---
