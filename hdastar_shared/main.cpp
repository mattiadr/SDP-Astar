#include <queue>
#include <iostream>
#include <thread>
#include <mutex>
#include <cfloat>
#include <barrier>

#include "../include/graph_utils/graph_utils.h"
#include "../include/stats/stats.h"


#define N_THREADS 16

#define myOpenSet openSets[threadId]
#define myOpenSetMutex openSetMutexes[threadId]


using namespace boost;


/** types **/

typedef struct {
	NodeId node;
	double fCost;
} NodeFCost;

const auto queue_comparator = [](const NodeFCost &a, const NodeFCost &b) { return a.fCost > b.fCost; };
typedef std::priority_queue<NodeFCost, std::vector<NodeFCost>, decltype(queue_comparator)> OpenSet;


/** globals **/

std::vector<std::thread> threads(N_THREADS);

// open sets
std::vector<std::unique_ptr<OpenSet>> openSets(N_THREADS);
std::vector<std::mutex> openSetMutexes(N_THREADS);

// came from
NodeId *cameFrom;

// cost to come
double *costToCome;
std::vector<std::mutex> costToComeMutexes(N_THREADS);

// best path
double bestPathWeight = DBL_MAX;
std::mutex bestPathMutex;

// termination
std::barrier barrier(N_THREADS);

// path reconstruction
std::vector<NodeId> path;
//std::vector<bool> finished(N_THREADS);
bool finished[N_THREADS];


/** functions **/

bool has_finished() {
	for (int i = 0; i < N_THREADS; i++) {
		if (!finished[i]) {
			return false;
		}
	}
	return true;
}

void hdastar_shared(const unsigned int threadId, const Graph &g, const NodeId &pathStart, const NodeId &pathEnd,
                    stats &stat) {
	NodeFCost nfc;
	while (true) {
		// termination condition
		std::unique_lock lock1(myOpenSetMutex);
		if (myOpenSet->empty()) {
			lock1.unlock();
			barrier.arrive_and_wait();
			// set finished flag
			finished[threadId] = myOpenSet->empty();
			barrier.arrive_and_wait();

			// check if all threads finished working, otherwise continue
			if (has_finished()) {
				break;
			} else {
				continue;
			}
		}

		// pop first from open set
		nfc = myOpenSet->top();
		myOpenSet->pop();
		lock1.unlock();

		if (nfc.fCost >= bestPathWeight)
			continue;

		// update bestPathWeight if we reached end of path
		if (nfc.node == pathEnd) {
			std::unique_lock lock(bestPathMutex);
			if (nfc.fCost < bestPathWeight) {
				bestPathWeight = nfc.fCost;
			}
			continue;
		}

		// iterate over neighbors
		double ctc;
		{
			std::unique_lock lock(costToComeMutexes[hash_node_id(nfc.node, N_THREADS)]);
			ctc = costToCome[nfc.node];
		}
		stat.addNodeVisited(threadId);
		for (auto neighbor: make_iterator_range(out_edges(nfc.node, g))) {
			NodeId target = neighbor.m_target;
			double weight = get(edge_weight, g, neighbor);
			double gCost = ctc + weight;
			double fCost = gCost + calc_h_cost(g, target, pathEnd);

			if (fCost < bestPathWeight) {
				unsigned int targetThread = hash_node_id(target, N_THREADS);
				std::unique_lock lock(costToComeMutexes[targetThread]);
				if (costToCome[target] > gCost) {
					costToCome[target] = gCost;
					cameFrom[target] = nfc.node;
					lock.unlock();
					{
						std::unique_lock lock2(openSetMutexes[targetThread]);
						openSets[targetThread]->push(NodeFCost{.node = target, .fCost = fCost});
					}
				}
			}
		}
	}
}

int path_reconstruction(const Graph &g, const NodeId &pathStart, const NodeId &pathEnd, stats &stat) {
	NodeId curr = pathEnd;
	while (true) {
		path.insert(path.begin(), curr);

		if (curr == pathStart)
			return 0;

		if (cameFrom[curr] == INVALID_NODE_ID) {
			std::cerr << "hdastar_shared: Error during path reconstruction: Node " << curr << " parent not found"
			          << std::endl;
			return 1;
		}
		curr = cameFrom[curr];
	}
}

int main(int argc, char *argv[]) {
	// print usage
	if (argc < 3) {
		std::cerr << "Usage: " << argv[0] << " FILENAME STARTING_SEED [N_SEEDS=1] [N_REPS=1]" << std::endl;
		return 1;
	}

	// parse command line parameters
	char *filename = argv[1];
	unsigned long seed;
	unsigned int nSeeds = 1;
	unsigned int nReps = 1;
	try {
		seed = std::stoul(argv[2]);
		if (argc >= 4)
			nSeeds = std::stoul(argv[3]);
		if (argc >= 5)
			nReps = std::stoul(argv[4]);
	} catch (std::invalid_argument const &e) {
		std::cerr << "Invalid command line parameter" << std::endl;
		return 2;
	}

	// read graph
	Graph g = read_graph(filename);
	unsigned int N = num_vertices(g);

	NodeId source, dest;

	// monte carlo simulation
	for (int k = 0; k < nSeeds; k++) {
		unsigned int local_seed = seed;
		for (int i = 0; i < nReps; i++) {
			stats s("HDA* Shared Memory", N_THREADS, filename, local_seed);

			// randomize seed every nReps runs
			if (i % nReps == 0)
				randomize_source_dest(seed, N, source, dest);

			std::cerr << "Repetition " << k << ", " << i << std::endl;
			s.timeStep("Start");

			// init open sets
			for (int j = 0; j < N_THREADS; j++) {
				openSets[j] = std::make_unique<OpenSet>(queue_comparator);
			}

			// init arrays
			cameFrom = new NodeId[N];
			std::fill_n(cameFrom, N, INVALID_NODE_ID);

			costToCome = new double[N];
			std::fill_n(costToCome, N, DBL_MAX);
			costToCome[source] = 0;

			// push first node and set cost to come
			NodeFCost nfc{.node = (NodeId) source, .fCost = 0};
			openSets[hash_node_id(source, N_THREADS)]->push(nfc);
			s.timeStep("Init done");

			// run threads
			for (int j = 0; j < N_THREADS; j++) {
				threads[j] = std::thread(hdastar_shared, j, ref(g), source, dest, ref(s));
			}

			for (int j = 0; j < N_THREADS; j++) {
				threads[j].join();
			}
			s.timeStep("Astar");

			int path_reconstruction_status = path_reconstruction(ref(g), source, dest, ref(s));

			// path reconstruction
			if (!path_reconstruction_status) {
				s.timeStep("Path reconstruction");
				s.printTimeStats();

				// print paths total cost
				double cost = 0;
				for (int j = 1; j < path.size(); j++) {
					cost += get(edge_weight, g, edge(path[j - 1], path[j], g).first);
				}
				std::cout << "Total cost: " << cost << std::endl;
				std::cout << "Total steps: " << path.size() << std::endl;

				s.setTotalCost(cost);
				s.setTotalSteps(path.size());
				s.dump_csv(path);
			}

			// free resources
			delete[] cameFrom;
			delete[] costToCome;

			// cleanup global variables
			bestPathWeight = DBL_MAX;
			path.clear();

			if (path_reconstruction_status)
				break;
		}
	}

	return 0;
}
