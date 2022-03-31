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
---
## Benchmark architecture
<img width="250" src="/docs/imgs/architecture.png">
The code is structured into a modular architecture with two models at its core.
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
