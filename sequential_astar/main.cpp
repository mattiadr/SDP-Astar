#include <iostream>
#include <utility>
#include <queue>
#include <cfloat> // Needed to use DBL_MAX

#include "../include/graph_utils/graph_utils.h"

using namespace boost;

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
	std::priority_queue<std::pair<unsigned int, unsigned int>, std::vector<std::pair<unsigned int, unsigned int>>, std::greater<>> openSet;
	std::set<unsigned int> closedSet;
	unsigned long long V = num_vertices(g);
	std::vector<double> costToCome(V, DBL_MAX);
	std::vector<unsigned int> cameFrom(V);

	costToCome[source] = 0;
	openSet.push(std::make_pair(source, 0));

	while (!openSet.empty()) {
		std::pair<unsigned int, unsigned int> curr_pair = openSet.top();
		unsigned int curr = curr_pair.first;
		openSet.pop();
		if (curr == target) {
			// Reconstructing path
			auto path = reconstruct_path(g, source, target, cameFrom);
			return path;
		}
		if (closedSet.find(curr) != closedSet.end())
			continue;
		closedSet.insert(curr);
		for (auto e: make_iterator_range(out_edges(curr, g))) {
			auto edge_w = get(edge_weight, g, e);
			if (closedSet.find(e.m_target) != closedSet.end())
				continue;
			double gcost = costToCome[curr] + edge_w;

			unsigned int curr_x = g[curr].x;
			unsigned int curr_y = g[curr].y;
			unsigned int edge_x = g[e.m_target].x;
			unsigned int edge_y = g[e.m_target].y;

			// The distance is the norm2 between the 2 points in the matrix
			double dist = sqrt(pow(edge_x - curr_x, 2) + pow(edge_y - curr_y, 2));

			double fcost = gcost + dist;
			if (gcost > costToCome[e.m_target])
				continue;
			cameFrom[e.m_target] = curr;
			costToCome[e.m_target] = gcost;
			openSet.push(std::make_pair(e.m_target, fcost));
		}
	}

	return std::make_pair(-1, std::vector<unsigned int>());
}

int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " FILENAME" << std::endl;
		return 1;
	}
	char* filename = argv[1];
	Graph g = read_graph(filename);

	unsigned int N = num_vertices(g);
	auto path_pair = astar_sequential(g, 0, N - 1);
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
	printf("FINISHED ASTAR");

	return 0;
}
