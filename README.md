BCP-MAPF
========

BCP-MAPF is an implementation of a branch-and-cut-and-price algorithm for the multi-agent path finding problem. It is described in the paper:

*Branch-and-Cut-and-Price for Multi-Agent Path Finding*. Edward Lam, Pierre Le Bodic, Daniel Harabor and Peter J. Stuckey. Computers & Operations Research, vol. 144, pp. 105809. 2022.

Please cite this article if you use this code for the multi-agent path finding problem or as a template for other branch-and-cut-and-price codes.

License
-------

BCP is released under the GPL version 3. See LICENSE.txt for further details.

Dependencies
------------

BCP is implemented in C++17 and is compiled using CMake, so you will need a recent compiler and a recent version of CMake. It is tested to run on Mac and Linux. It has not been tested on Windows.

BCP uses SCIP 7.0.3 for branch-and-bound. Source code to SCIP is available free (as in 🍺) for academic use. BCP calls Gurobi or CPLEX for solving the linear relaxation. [Gurobi](https://www.gurobi.com/downloads/end-user-license-agreement-academic/) and [CPLEX](https://community.ibm.com/community/user/datascience/blogs/xavier-nodet1/2020/07/09/cplex-free-for-students) are commercial software but provide free binaries under an academic license. BCP is tested with Gurobi 10.0.1 and CPLEX 20.1.0.

Compiling
---------

1. Download the source code to BCP-MAPF by cloning this repository and all its submodules.
```
git clone --recurse-submodules https://github.com/ed-lam/bcp-mapf.git
cd bcp-mapf
```

2. Download the [SCIP Optimization Suite 7.0.3](https://www.scipopt.org/index.php#download) into the `bcp-mapf` directory and extract it. You should find the subdirectory `scipoptsuite-7.0.3/scip/src`.
```
tar xf scipoptsuite-7.0.3.tgz
```

3. Download the [Boost C++ libraries](https://www.boost.org/users/download/) if not already provided by your system.
```
wget https://boostorg.jfrog.io/artifactory/main/release/1.79.0/source/boost_1_79_0.tar.gz
tar xf boost_1_79_0.tar.gz
```

4. Compile BCP-MAPF.

    1. If using Gurobi, locate its directory, which should contain the `include` and `lib` subdirectories. Copy the main directory into the command below.
    ```
    cmake -DGUROBI_DIR={PATH TO GUROBI DIRECTORY} .
    cmake --build .
    ```

    2. If using CPLEX, locate the subdirectory `cplex` and copy this directory into the command below.
    ```
    cmake -DCPLEX_DIR={PATH TO cplex SUBDIRECTORY} .
    cmake --build .
    ```

    If Boost is missing, download Boost in step 3 and append `-DBOOST_ROOT={PATH TO BOOST DIRECTORY}` to the first `cmake` command above.

    To compile with a multi-core CPU with N cores, append `-j N` to the second `cmake` command above.

Usage
-----

After compiling, run BCP with:
```
./bcp-mapf {PATH TO INSTANCE}
```

You can also set a time limit in seconds:
```
./bcp-mapf —-time-limit={TIME LIMIT} {PATH TO INSTANCE}
```

BCP can be run as a bounded suboptimal algorithm by setting an optimality gap. For example, enter `0.1` for a 10% optimality gap.
```
./bcp-mapf —-gap-limit={OPTIMALITY GAP} {PATH TO INSTANCE}
```

The Moving AI benchmarks can be found in the `instances/movingai` directory. There is (usually) a total of 1000 agents in each instance file, and the user can specify how many of the first N agents to run. For example, you can run an instance with only the first 50 agents:
```
./bcp-mapf --time-limit=30 --agent-limit=50 instances/movingai/Berlin_1_256-random-1.scen
```

The optimal solution (or feasible solution if a time limit or gap limit is reached) will be saved into the `outputs` directory.

Contributing
------------

We welcome code contributions and scientific discussion subject to [Monash University’s equal opportunity and harassment policies](https://www.monash.edu/about/diversity-inclusion/staff/equal-opportunity).

Authors
-------

BCP is invented by Edward Lam with assistance from Pierre Le Bodic, Daniel Harabor and Peter J. Stuckey. Edward can be reached at [ed-lam.com](https://ed-lam.com).