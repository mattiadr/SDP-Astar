# SDP-Astar

PoliTO System and Device Programming project Quer 1 (The Path-Planning Algorithm A*)

- D'Andrea Giuseppe s303378
- De Rosa Mattia 303379

## Folders structure

- `binaries` - Precompiled binaries for windows and linux
- `data` - Graph input files and raw output files
- `documentation` - Documentation markdown source
- `graph_generation` - C++ source code for k-neighbors graph generation
- `hdastar_message_passing` - C++ source code for Hash Distributed A* with message passing
- `hdastar_shared` - C++ source code for Hash Distributed A* with shared memory
- `include`
  - `boost_1_80_0` - Minimal version of Boost C++ Library v1.80.0
  - `graph_utils` - C++ source code for common operations for the different algorithms
  - `stats` - C++ source code for stats gathering helper class
- `scripts` - Python helper scripts
  - `launcher.py` - Wrapper to run multiple versions of A*
  - `osm_to_graph.py` - Script to convert OpenStreetMap XML files to graphs
- `sequential_astar` - C++ source code for sequential A*

## Graph generation

Multiple helper scripts have been used to generate the graphs used to test the algorithms.

- `graph_generation/main.cpp`: generate a random 2D graph, in a square grid of size ___S*S___ with __*N*__ nodes, each connected with its __*K*__ nearest neighbors.
  - Usage: `graph_generation.exe S N [K]`. If __*K*__ is omitted is calculated as $ K = 2*e*log(n) $
- `scripts/osm_to_graph.py`: python script to convert OpenStreetMap XML files to graph text files.
  - Usage: `python osm_to_graph_py INPUT_FILE`
  - Input files for this script have been obtained thanks to [Overpass Api](https://wiki.openstreetmap.org/wiki/Overpass_API) and [Overpass Turbo](https://overpass-turbo.eu/)

## A* algorithms

The different algorithms are contained in the following folders:

- `sequential_astar`: Sequential version of A*
- `hdastar_message_passing`: Parallel version of Hash Distributed A* that uses message_passing and barriers to synchronize threads
- `hdastar_shared`: Parallel version of Hash Distributed A* that uses shared memory and barriers to synchronize threads

## Build

Precompiled binaries are provided for windows_x86_64 and for linux_x86_64 in the folder `binaries`

It is suggested to build the project with _gcc_ on linux or with _MSVC_ on Windows. _MinGW_ on Windows causes issues with the barriers and the parallel versions don't work properly.

The project contains `CMakeLists.txt` which can be used to build the different versions of the algorithm.

Targets:
- `graph_generation`
- `sequential_astar`
- `hdastar_message_passing`

Example:
```
cmake -S SDP-Astar/ -B build_folder/
cmake --build build_folder/ --target sequential_astar -j 12
```

## Run

All versions of A* have the same usage: `./executable FILENAME STARTING_SEED [N_SEEDS=1] [N_REPS=1]`

The first two parameters are mandatory and have to be the name of the graph file and the seed used to randomly generate the start and end nodes. The following two parameters are used to execute the algorithm multiple times with different seeds and multiple time per seed respectively.

Examples:
- `sequential_astar berlin.txt 24` executes sequential A* once with given seed
- `hdastar_shared turin.txt 1234 5 3` executes shared memory A* a total of 15 times, 3 for each different seed, starting from the given one

Execution results are dumped in a csv file (`AstarReport.csv`) for every run performed, containing multiple statistics, including found path, number of steps, total weight of path and execution time for every phase.

A summary of these results is also printed in stdout for every run.

## External resources

The project includes part of the [boost C++ library](https://www.boost.org/). Version 1.80.0 of Boost has been used to code and test the algorithms and a minimal version of boost including only the required classes has been included in the zip file.

If you want to manually add the library download it from the official site https://www.boost.org/users/download/ and unzip in the include directory. If the version is not the 1.80.0 or the folder is not called `boost_1_80_0` the `CMakeLists.txt` should be updated with the new location of the library.
