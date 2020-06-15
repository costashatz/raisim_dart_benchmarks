# RaiSim vs DART timing Benchmarks

This repo contains a few benchmarks that compare [RaiSim](https://github.com/leggedrobotics/raisimLib) to [DART](https://github.com/dartsim/dart) robotic simulators.


## Benchmarks

All simulations are ran for **10 seconds of simulated time** with an integration time-step of **0.001 seconds**. We replicated each experiment **10 times** on a laptop CPU.

### Quick Analysis

The exact timings are not important, but the relative timings showcase that RaiSim is consistently faster than DART (ranging from 5-30x times faster depending on the number of contacts and the scene). It is normal that RaiSim is faster than DART when there are contacts, since RaiSim implements a novel idea about contact dynamics solving that is supposed to be faster. It is interesting to note though that RaiSim is much faster than DART even in contact-free scenarios by at least 5x.

### ANYmal Quadruped

#### Fixed-base Simulation

You can find this benchmark in `anymal_benchmark.cpp` file. The simulation consists of an ANYmal quadruped robot fixed with a weld joint, and that has to keep the current joint configuration. The robot is torque-controlled with a PD controller. There are **no contacts** involved.

##### Bechmark results

**RaiSim average running time for 10s of simulation:** 0.0405115s (*real-time factor:* 246.844)

**DART average running time for 10s of simulation:** 0.215082s (*real-time factor:* 46.4938)

RaiSim is **5.30917** faster than DART.

#### Floating-base Simulation

You can find this benchmark in `anymal_contacts_benchmark.cpp` file. The simulation consists of four (4) ANYmal quadruped robots that have to keep the current joint configuration in order to stay standing (interacting with a ground). The robots are torque-controlled with a PD controller. This simulation involves contacts.

##### Bechmark results

**RaiSim average running time for 10s of simulation:** 0.438723s (*real-time factor:* 22.7934)

**DART average running time for 10s of simulation:** 3.55465s (*real-time factor:* 2.81322)

RaiSim is **8.10225** faster than DART.

### KUKA IIWA

You can find this benchmark in `iiwa_benchmark.cpp` file. The simulation consists of a KUKA IIWA 14 robot fixed with a weld joint, and that has to achieve a certain joint configuration starting from an upright position. The robot is torque-controlled with a PD controller. There are **no contacts** involved.

##### Bechmark results

**RaiSim average running time for 10s of simulation:** 0.0237632s (*real-time factor:* 420.819)

**DART average running time for 10s of simulation:** 0.122529s (*real-time factor:* 81.6136)

RaiSim is **5.15623** faster than DART.

### Six-legged Robot

You can find this benchmark in `pexod_benchmark.cpp` file. The simulation consists of four (4) simple 6-legged robots ([pexod](https://www.resibots.eu/photos.html#pexod-robot)) that have to keep the current joint configuration in order to stay standing (interacting with a ground). The robots are torque-controlled with a PD controller. This simulation involves contacts.

##### Bechmark results

**RaiSim average running time for 10s of simulation:** 0.716966s (*real-time factor:* 13.9477)

**DART average running time for 10s of simulation:** 9.80957s (*real-time factor:* 1.01941)

RaiSim is **13.6821** faster than DART.

### Simple Objects

You can find this benchmark in `objects_benchmark.cpp` file. The simulation consists of multiple spheres and boxes that interact with each other and the ground. There is no controller; the objects are just following gravity. This simulation involves many contacts.

##### Bechmark results

**RaiSim average running time for 10s of simulation:** 1.21893s (*real-time factor:* 8.20393)

**DART average running time for 10s of simulation:** 37.7686s (*real-time factor:* 0.26477)

RaiSim is **30.9851** faster than DART.

## Compile and Run the Benchmarks

- Install DART (we need also the Bullet-Collision) and RaiSim (in a local path)
- `mkdir build`
- `cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/path/where/RaiSim/is/installed`
- `make -j`
- The executables have the same names as the source files. They should be located inside the `build` folder.
