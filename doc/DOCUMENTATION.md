
![](./imgs/polito_logo_2021_blu.jpg)
# The Path-Planning Algorithm A*

__System and Device Programming project Quer 1__

D'Andrea Giuseppe s303378

De Rosa Mattia s303379

## Problem description

The proposed problem requires to compare the performance of different implementations of the path-planning algorithm A*.

We chose to develop a sequential version to use for reference and 2 parallel versions. One based on shared memory and
the other one on message passing.

The main problems of the parallel versions compared to the sequential ones are the exploration of already explored nodes
and the termination condition.

The first problem is solved assigning each node to a different thread using an hash function so that each thread can
keep track of the already explored nodes and avoid unnecessary work.

The second problem has proven to be the main obstacle we found when implementing the parallel versions of A*. In the
sequential version once a path is found we are sure that is the best path we can find given the start and end points. In
the parallel version instead we cannot be sure of the order of exploration of the nodes, so we have to keep
exploring until all the paths remaining are surely worse than the best found. In simple graphs this will allow the
sequential version to explore a minimal part of the graph resulting in better performance than the parallel one.

## Implementation

The algorithms are implemented in C++ with additional boost libraries:
- `boost/graph/adjacency_list` - Graph library used to store the graph as an adjacency list.
- `boost/lockfree/queue` - Lock-free queues used to implement message-passing queues

In each version of the algorithm we implemented the openSets with a `std::priority_queue` sorted by the estimated path
length of that node. 

The cost to come to each node and the parent lookup table are stored in dynamically allocated arrays. We also tried to
use `std::unordered_map` to lower memory usage, but the loss of performance was not worth it. 

### Input file

The input file represents a 2D map of nodes, each with a couple of coordinates _(x, y)_ that will be imported as an
undirected weighted graph.

The graph files are provided as text files with a specific format:

```
n_nodes
coordx_1 coordy_1
.
.
.
coordx_n coordy_n
index_node_j index_node_k weight
.
.
.
index_node_w index_node_z weight
```

In the first line there is the number of nodes, after that n lines with the coordinates of each node, and then there is
the list of each edge using the indexes of the nodes in the list and the weight of the edge.

The text file is parsed and a Graph object is created including all the information provided in the input file.

### Heuristic function

We chose the $L_2$ norm distance from vertex __i__ to destination vertex __d__ as the heuristic function. We can compute
it given the coordinates of the nodes with the formula: $$ h(i) = \sqrt{ (d_x - i_x)^2 + (d_y - i_y)^2 } $$
The weight of the edges in the graph is computed with the same function. This heuristic can never overestimate the
cost and follows the triangle inequality, in other words it is _admissible_ and _consistent_, so we can be sure that in
the sequential version, the first path found will be the best one.

### Parallelism
The parallelism is obtained using `std::thread`, the implementation of threads of C++ standard library. Synchronization
is managed with `std::barrier` for the message passing version and with `std::counting_semaphore` for the shared memory
version.

Message queues are implemented using boost lock-free queues or with single consumer multi producer queues which
use `std::mutex` to protect from race conditions when adding to the queue.

Single consumer multi producer queue `scmp_queue.h` has been implemented as an alternative to boost lock-free queues. We
use two separate queues for reading and for writing, switching them when the reading queue is empty. In this way
reading from the queue should be lock-free in most cases.

[//]: # (TODO Add something on the shared version)

### Termination condition

The basic termination condition for a parallel implementation of A* could be to stop every thread as soon as every
openSet is empty. Without additional constraints this would lead to the exploration of all the graph. To avoid this, we 
keep track of the best path found so far, and we add a node to the openSet only if its estimated cost is less than the
best path. In this way we can be sure to explore a node only if there is the possibility to find a better path, and this
is acceptable only if the heuristic function never overestimates the goal (_admissible_).

Every time a thread find that its openSet is empty, it hits a barrier, waiting for the other threads. When everyone hit
the barrier, they can check if every other thread has finished its work, in that case we know that the best path has
been found and can be reconstructed.
