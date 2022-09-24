#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

using namespace boost;

typedef struct {
	unsigned int x;
	unsigned int y;
} VertexPosition;

typedef property<edge_weight_t, double> EdgeWeightProperty;
typedef adjacency_list<vecS, vecS, undirectedS, VertexPosition, EdgeWeightProperty> Graph;
typedef std::pair<unsigned int, unsigned int> Edge;

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

#endif
