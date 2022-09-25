#include <queue>

#include "../include/graph_utils/graph_utils.h"

using namespace boost;

typedef unsigned int NodeId;
typedef std::pair<NodeId, double> NodeFCost;

void
hdastar_distributed(unsigned int thread_id, unsigned int n_threads, Graph g) {
	auto comp = [](NodeFCost a, NodeFCost b) { return a.second > b.second; };
	std::priority_queue<NodeFCost, std::vector<NodeFCost>, decltype(comp)> openSet;
	std::unordered_map<NodeId, double> costToCome;
	std::unordered_map<NodeId, NodeId> cameFrom;

}

int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " FILENAME" << std::endl;
		return 1;
	}
	char* filename = argv[1];
	Graph g = read_graph(filename);
	unsigned int N = num_vertices(g);


}
