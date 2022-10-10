#include <iostream>
#include <utility>
#include <queue>
#include <cfloat> // Needed to use DBL_MAX

#include "../include/graph_utils/graph_utils.h"
#include "../include/stats/stats.h"

using namespace boost;

typedef std::pair<unsigned int, double> NodeFCost;

stats s;

// Reconstruct path from graph and list of costs to nodes
std::pair<double, std::vector<unsigned int>>
reconstruct_path(const Graph &g, unsigned int source, unsigned int target, std::vector<unsigned int> &cameFrom) {
	double path_weight = 0;
	std::vector<unsigned int> path;
	path.emplace_back(target);
	unsigned int current = target;
	while (current != source) {
		current = cameFrom[current];
		path_weight += get(edge_weight, g, edge(current, target, g).first);
		target = current;
		path.insert(path.begin(), current);
	}
	return std::make_pair(path_weight, path);
}

// Find the best path from source to target node. Prints the results on file
std::pair<double, std::vector<unsigned int>>
astar_sequential(const Graph &g, unsigned int source, unsigned int target) {
	auto comp = [](NodeFCost a, NodeFCost b) { return a.second > b.second; };
	std::priority_queue<NodeFCost, std::vector<NodeFCost>, decltype(comp)> openSet;
	std::set<unsigned int> closedSet;
	unsigned long long V = num_vertices(g);
	std::vector<double> costToCome(V, DBL_MAX);
	std::vector<unsigned int> cameFrom(V);

	costToCome[source] = 0;
	openSet.push(NodeFCost(source, 0));

	while (!openSet.empty()) {
		NodeFCost curr_pair = openSet.top();
		unsigned int curr = curr_pair.first;
		openSet.pop();
		if (curr == target) {
			s.timeStep("A*");
			// Reconstructing path
			auto path = reconstruct_path(g, source, target, cameFrom);
			s.timeStep("Path reconstruction");
			return path;
		}
		if (closedSet.find(curr) != closedSet.end())
			continue;
		closedSet.insert(curr);
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

	return std::make_pair(-1, std::vector<unsigned int>());
}

int main(int argc, char *argv[]) {
	if (argc < 3) {
		std::cerr << "Usage: " << argv[0] << " FILENAME SEED" << std::endl;
		return 1;
	}
	char* filename = argv[1];
	char *parseEnd;

	unsigned int seed = strtol(argv[2], &parseEnd, 10);
	if (*parseEnd != '\0') {
		std::cerr << "SEED must be a number, got " << argv[2] << " instead" << std::endl;
		return 2;
	}

	s.timeStep("Start");
	Graph g = read_graph(filename);

	unsigned int N = num_vertices(g);
	NodeId source, dest;

	randomize_source_dest(seed, N, source, dest);
	s.timeStep("Read graph");

	auto path_pair = astar_sequential(g, source, dest);
	auto path_weight = path_pair.first;
	auto path = path_pair.second;

	if (path_weight < 0) {
		std::cout << "Unable to find a path" << std::endl;
		return 2;
	}

	// Print path
	std::cout << path_weight << std::endl;
	char *fout_filename = (char *) malloc((strlen(filename) + 8) * sizeof(char));
	sprintf(fout_filename, "%s.solution", filename);
	FILE *fout = fopen(fout_filename, "w");
	for (auto el: path) {
		std::cout << el << " -> ";
		fprintf(fout, "%d\n", el);
	}
	fclose(fout);
	std::cout << std::endl;
	std::cout << "FINISHED ASTAR" << std::endl;

	s.printTimeStats();

	return 0;
}
