# SDP-Astar
PoliTO System and Device Programming project Quer 1 (The Path-Planning Algorithm A*)

- D'Andrea Giuseppe s303378
- De Rosa Mattia 303379

## Graph generation
Different helper scripts are used to generate the graphs used to test the algorithms.
- `graph_generation/main.cpp`: generate a random 2D graph, in a squarer grid of size ___S*S___ with __*N*__ nodes, each connected with its __*K*__ nearest neighbors.
  - Usage: `graph_generation.exe S N [K]`. If __*K*__ is omitted is calculated as $ K = 2*e*log(n) $
- `scripts/osm_to_graph.py`: python script to convert OpenStreetMap XML files to graph text files.
  - Usage: `python osm_to_graph_py INPUT_FILE`

## Astar algorithms
The different algorithms are contained in the following folders:
- `sequential_astar`: Sequential version of astar
- `hdastar_message_passing`: Parallel version of hdastar that uses message_passing and barriers to synchronize threads 
- `hdastar_shared`: Parallel version of hdastar that use shared memory and barriers to synchronize threads

## Build
Precompiled binaries are provided for windows_x86_64 and for linux_x86_64 in the folder:

[//]: # (TODO: generate precompiled binaries and update folder)

It is suggested to build the project with _gcc_ on linux or with _MSVC_ on Windows. _MinGW_ on Windows causes problems with the barriers and the parallel versions don't work properly.

The project contains the `CMakeLists.txt` to be used with cmake to build the different versions of the algorithm.

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
Once compiled the algorithms should be executed with the filename of the graph input file and a seed. The seed is used to randomly choose the start and end points in the graph.

Example: `sequential_astar berlin.txt 24`

[//]: # (TODO add something on the expected output)

## External resources
The project includes some external classes from the boost library. The version 1.80.0 of Boost has been used to code and test the algorithms and a minimal version of boost including only the required classes has been included in the zip file.

If you want to manually add the library download it from the official site https://www.boost.org/users/download/ and unzip in the include directory. If the version is not the 1.80.0 or the folder is not called `boost_1_80_0` the `CMakeLists.txt` should be updated with the new location of the library.
