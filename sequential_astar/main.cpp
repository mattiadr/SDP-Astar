#include <iostream>
#include <utility>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphviz.hpp>
#include <queue>
#include <cfloat> // Needed to use DBL_MAX

using namespace boost;

typedef struct {
	unsigned int x;
	unsigned int y;
} VertexPosition;

typedef property<edge_weight_t, double> EdgeWeightProperty;
typedef adjacency_list<vecS, vecS, undirectedS, VertexPosition, EdgeWeightProperty> Graph;
typedef std::pair<unsigned int, unsigned int> Edge;

char *filename;

// Reconstruct path from graph and list of costs to nodes
std::vector<unsigned int>
reconstruct_path(const Graph &g, unsigned int source, unsigned int target, std::vector<unsigned int> &cameFrom) {
	std::vector<unsigned int> path;
	path.emplace_back(target);
	unsigned int current = target;
	while (current != source) {
		current = cameFrom[current];
		path.insert(path.begin(), current);
	}
	return path;
}

// TODO: check for unsigned int / unsigned long long
// Find the best path from source to target node. Prints the results on file
void astar_sequential(const Graph &g, unsigned int source, unsigned int target) {
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
			std::vector<unsigned int> path = reconstruct_path(g, source, target, cameFrom);
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
			return;
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
}

// Read graph from file. Graph should be generated from graph_generation script
Graph read_graph(char *fin_filename) {
	FILE *fin = fopen(fin_filename, "r");
	unsigned int n_nodes;
	fscanf(fin, "%d", &n_nodes);
	Graph g(n_nodes);
	unsigned int x, y;
	for (int i = 0; i < n_nodes; i++) {
		fscanf(fin, "%d %d", &x, &y);
		g[i].x = x;
		g[i].y = y;
	}
	unsigned int node1, node2;
	double weight;
	while (fscanf(fin, "%d %d %lf", &node1, &node2, &weight) == 3) {
		add_edge(node1, node2, EdgeWeightProperty(weight), g);
	}
	return g;
}

// Print graph on the provided output stream using graphviz.
// The graph that can be rendered on: https://dreampuf.github.io/GraphvizOnline
void print_graph(Graph g, std::ostream &os) {
	dynamic_properties dp;
	dp.property("node_id", get(vertex_index, g));
	dp.property("weight", get(edge_weight, g));
	write_graphviz_dp(os, g, dp);
}

int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " FILENAME" << std::endl;
		return -1;
	}
	filename = argv[1];
	Graph g = read_graph(filename);
//    print_graph(g, std::cout);

	unsigned int N = num_vertices(g);
	astar_sequential(g, 0, N - 1);

	return 0;
}
