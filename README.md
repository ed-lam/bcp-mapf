BCP
===

BCP is an implementation of a branch-and-cut-and-price model of the multi-agent path finding problem. It is described in the following papers:

- Branch-and-Cut-and-Price for Multi-Agent Pathfinding. E. Lam, P. Le Bodic, D. Harabor, P. J. Stuckey. IJCAI 2019.
- New Valid Inequalities in Branch-and-Cut-and-Price for Multi-Agent Path Finding. E. Lam, P. Le Bodic. ICAPS 2020.

Please cite these articles if you use this code for the multi-agent path finding problem or as a template for other branch-and-cut-and-price codes.

License
-------

BCP is released under the GPL version 3. See LICENSE.txt for further details. 

Dependencies
------------

BCP is implemented in C++17 and is compiled using CMake, so you will need a recent compiler and a recent version of CMake. It is tested with Clang 11 on Mac and GCC 10 on Linux. It has not been tested on Windows.

BCP calls SCIP for branch-and-bound and calls CPLEX for solving the linear relaxation.

Source code to SCIP is available free (as in beer) strictly for academic use. BCP is tested with SCIP 7.0.2. Download the [SCIP Optimization Suite 7.0.2](https://scip.zib.de) and extract it into the root of this repository. You should find the subdirectory `scipoptsuite-7.0.2/scip/src`.

CPLEX is commercial software but has binaries available free under an [academic license](https://community.ibm.com/community/user/datascience/blogs/xavier-nodet1/2020/07/09/cplex-free-for-students). BCP is tested with CPLEX 12.10. You should find the subdirectory `cplex`.

If CPLEX is not available, SoPlex from the SCIP Optimization Suite can be used instead but this option is not supported.

Compiling
---------

Download the source code by cloning this Git repository and all its submodules:
```
git clone --recurse-submodules https://github.com/ed-lam/bcp-mapf.git
```

Locate the `cplex` subdirectory inside wherever you downloaded the CPLEX binaries. Compile BCP using CMake:
```
cd bcp-mapf
mkdir build
cd build
cmake -DCPLEX_DIR={PATH TO cplex SUBDIRECTORY} ..
cmake --build .
```

If you use a custom compiler, you will need to tell CMake where the compiler is:
```
cmake -DCPLEX_DIR={PATH TO cplex SUBDIRECTORY} -DCMAKE_C_COMPILER={PATH TO C COMPILER} -DCMAKE_CXX_COMPILER={PATH TO C++ COMPILER} ..
```

If you have a multi-core CPU with N cores, you can perform a parallel compile by running the following command instead, replacing N with the number of cores:
```
cmake --build . -j N
```

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

BCP can be run as a bounded suboptimal algorithm by setting an optimality gap, calculated as (upper bound - lower bound) / lower bound. For example, enter `0.1` for a 10% optimality gap.
```
./bcp-mapf —-gap-limit={OPTIMALITY GAP} {PATH TO INSTANCE}
```

Benchmark instances can be found in the `movingai_2018` and `movingai_2019` directories. Example:
```
./bcp-mapf --time-limit=30 ../instances/movingai_2018/dao_maps/lak503dmap-100agents-49.scen
```

The 2019 instances are organised differently. There is (usually) a total of 1000 agents in each instance file, and the user can specify how many of the first N agents to run. For example, you can run an instance with only the first 50 agents:
```
./bcp-mapf --time-limit=30 --agents-limit=50 ../instances/movingai_2019/den520d-random-1.scen
```

The optimal solution (or feasible solution if a time limit or gap limit is reached) will be saved into the `outputs` directory.

Contributing
------------

We welcome code contributions and scientific discussion subject to [Monash University’s equal opportunity and harassment policies](https://www.monash.edu/about/diversity-inclusion/staff/equal-opportunity).

Authors
-------

BCP is invented by Edward Lam with assistance from Pierre Le Bodic, Daniel Harabor and Peter J. Stuckey. Edward can be reached at [ed-lam.com](https://ed-lam.com).