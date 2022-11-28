#include <iostream>
#include <utility>
#include <queue>
#include <cfloat> // Needed to use DBL_MAX

#include "../include/graph_utils/graph_utils.h"
#include "../include/stats/stats.h"

using namespace boost;

typedef std::pair<unsigned int, double> NodeFCost;

// Reconstruct path from graph and list of costs to nodes
std::pair<double, std::vector<unsigned int>>
reconstruct_path(const Graph &g, unsigned int source, unsigned int target, const NodeId *cameFrom,
                 const double *costToCome, const stats &s) {
	std::vector<unsigned int> path;
	path.emplace_back(target);
	unsigned int current = target;
	while (current != source) {
		if (cameFrom[current] == INVALID_NODE_ID) {
			std::cerr << "sequential_astar: Error during path reconstruction: Node " << current << " parent not found"
			          << std::endl;
			return std::make_pair(0, path);
		}
		current = cameFrom[current];
		target = current;
		path.insert(path.begin(), current);
	}
	return std::make_pair(costToCome[target], path);
}

// Find the best path from source to target node. Prints the results on file
std::pair<double, std::vector<unsigned int>>
astar_sequential(const Graph &g, unsigned int source, unsigned int target, stats &s) {
	auto comp = [](NodeFCost a, NodeFCost b) { return a.second > b.second; };
	std::priority_queue<NodeFCost, std::vector<NodeFCost>, decltype(comp)> openSet;
	std::set<unsigned int> closedSet;
	unsigned long long V = num_vertices(g);
	double *costToCome = new double[V];
	std::fill_n(costToCome, V, DBL_MAX);
	NodeId *cameFrom = new NodeId[V];
	std::fill_n(cameFrom, V, INVALID_NODE_ID);

	costToCome[source] = 0;
	openSet.push(NodeFCost(source, 0));

	while (!openSet.empty()) {
		NodeFCost curr_pair = openSet.top();
		unsigned int curr = curr_pair.first;
		openSet.pop();
		if (curr == target) {
			s.timeStep("Astar");
			// Reconstructing path
			auto path = reconstruct_path(ref(g), source, target, cameFrom, costToCome, ref(s));
			s.timeStep("Path reconstruction");
			return path;
		}
		if (closedSet.find(curr) != closedSet.end())
			continue;
		closedSet.insert(curr);
		s.addNodeVisited(0);
		for (auto e: make_iterator_range(out_edges(curr, g))) {
			auto edge_w = get(edge_weight, g, e);
			if (closedSet.find(e.m_target) != closedSet.end())
				continue;
			double gCost = costToCome[curr] + edge_w;
			double fCost = gCost + calc_h_cost(g, e.m_target, target);
			if (gCost > costToCome[e.m_target])
				continue;
			cameFrom[e.m_target] = curr;
			costToCome[e.m_target] = gCost;
			openSet.push(NodeFCost(e.m_target, fCost));
		}
	}

	delete[] costToCome;
	delete[] cameFrom;

	return std::make_pair(-1, std::vector<unsigned int>());
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
	for (int i = 0; i < nSeeds * nReps; i++) {
		// randomize seed every nReps runs
		if (i % nReps == 0)
			randomize_source_dest(seed, N, source, dest);

		std::cerr << "Repetition " << i / nReps << ", " << i % nReps << std::endl;
		stats s("A*", 1, filename, seed);
		s.timeStep("Start");

		auto path_pair = astar_sequential(ref(g), source, dest, ref(s));
		auto path_weight = path_pair.first;
		auto path = path_pair.second;

		if (path_weight < 0) {
			return 3;
		}

		// Print path
		s.printTimeStats();

		std::cout << "Total cost: " << path_pair.first << std::endl;
		std::cout << "Total steps: " << path.size() << std::endl;

		s.setTotalCost(path_pair.first);
		s.setTotalSteps(path_pair.second.size());
		s.dump_csv(path_pair.second);
	}

	return 0;
}
