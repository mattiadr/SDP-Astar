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
std::vector<bool> finished(N_THREADS);


/** functions **/

bool has_finished() {
	for (int i = 0; i < N_THREADS; i++) {
		if (!finished[i]) {
			return false;
		}
	}
	return true;
}

void hdastar_shared(const unsigned int threadId, const Graph &g, const NodeId &pathStart, const NodeId &pathEnd, stats &stat) {
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

void path_reconstruction(const Graph &g, const NodeId &pathStart, const NodeId &pathEnd, stats &stat) {
	NodeId curr = pathEnd;
	while (true) {
		path.insert(path.begin(), curr);

		if (curr == pathStart)
			return;

		curr = cameFrom[curr];
		if (curr == INVALID_NODE_ID) {
			std::cerr << "Error during path reconstruction: Node " << curr << " parent not found" << std::endl;
			return;
		}
	}
}

int main(int argc, char *argv[]) {
	// print usage
	if (argc < 3) {
		std::cerr << "Usage: " << argv[0] << " FILENAME SEED" << std::endl;
		return 1;
	}

	// read seed
	char *parseEnd;
	unsigned long seed = strtol(argv[2], &parseEnd, 10);
	if (*parseEnd != '\0') {
		std::cerr << "SEED must be a number, got " << argv[2] << " instead" << std::endl;
		return 2;
	}

	// read graph
	char *filename = argv[1];
	stats s("HDA* Shared Memory", N_THREADS, filename, seed);
	s.timeStep("Start");
	Graph g = read_graph(filename);

	// generate source and dest from seed
	unsigned int N = num_vertices(g);
	NodeId source, dest;
	randomize_source_dest(seed, N, source, dest);
	s.timeStep("Read graph");

	// init open sets
	for (int i = 0; i < N_THREADS; i++) {
		openSets[i] = std::make_unique<OpenSet>(queue_comparator);
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
	for (int i = 0; i < N_THREADS; i++) {
		threads[i] = std::thread(hdastar_shared, i, ref(g), source, dest, ref(s));
	}

	// join threads
	for (int i = 0; i < N_THREADS; i++) {
		threads[i].join();
	}
	s.timeStep("Astar");

	// path reconstruction
	path_reconstruction(ref(g), source, dest, ref(s));
	s.timeStep("Path reconstruction");
	s.printTimeStats();

	// free resources
	delete[] cameFrom;
	delete[] costToCome;

	// print paths total cost
	double cost = 0;
	for (int i = 1; i < path.size(); i++) {
		cost += get(edge_weight, g, edge(path[i - 1], path[i], g).first);
	}
	std::cout << "Total cost: " << cost << std::endl;
	std::cout << "Total steps: " << path.size() << std::endl;

	s.setTotalCost(cost);
	s.setTotalSteps(path.size());
	s.dump_csv(path);
}
