# arena-bench
This repository provides the code used in our paper [Arena-Bench: A Benchmarking Suite for Obstacle Avoidance Approaches in Highly Dynamic Environments](). The scop of this project is to benchmark performance of mobile robots in dynamic simulation environments.
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
| - `jackal` [dff-drive] <br> - `burger` [car-like] <br> - `robotino` (rto) [holonomic] | - `small\_warehouse` <br> - `map2` <br> - `map5` | - `teb` <br> - `dwa` <br> - `mpc` <br> - `rosnav` <br> - `cadrl` <br>  | - `5` <br> - `10` |

The following clips show examples of the implemented scenarios:


---
## Benchmark architecture
<img src="/docs/imgs/architecture.jpg">
The repository is structured into a modular architecture with four models at its core.

- [Arena-Tools](https://github.com/ignc-research/arena-tools):
this module is being responsible to create custom worlds, scenarios and obstacles and adding those to the respictive location. It is possible to use arena-bench without arena-tools as a number of worlds, scenarios and obstacles have already been included into the world environments, including the all scenarios and worlds to reproduce this paper.

- [Arena-Rosnav](https://github.com/ignc-research/arena-rosnav):
this module contains a custom task-generator and the 2D simulation engine Flatland, enabeling robot simulation in 2D Flatland environemnts (note that this repository has been added recently to the arena-bench repository, and is therefore not part of the original paper).

- [Arena-Rosnav-3D](https://github.com/ignc-research/arena-rosnav-3D):
this module contains a custom task-generator and the 3D simulation engine Gazebo, enabeling robot simulation in 3D Gazebo environments.

- [Arena-Evaluation](https://github.com/ignc-research/arena-evaluation):
this module can be used to record and evaluate simulation runs and visualize Robot performance by creating qualitative and quantitative plots.

> **NOTE**: All modular components have their individual repository and are currently being extended with additional features

---
## Running the Benchmark
### Prerequisites
Below is the software we used. We cannot guarantee older versions of the software to work. Yet, newer software is most likely working just fine.

| Software      | Version        |
| ------------- | -------------- |
| OS            | Ubuntu 20.04.4 |
| Python        | 3.8.10         |

### Installation
To install the repo run:
```bash
wget https://raw.githubusercontent.com/ignc-research/arena-bench/main/setup.sh -O - | bash
```
### Manually running the benchmark
---

## Thanks
https://github.com/zal/simenvbenchmark#prerequisites
