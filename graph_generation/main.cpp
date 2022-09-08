#include <random>
#include <iostream>
#include <limits>
#include <cmath>
#include <set>
#include <sstream>

typedef std::pair<int, int> Vertex;

const double MAX_DOUBLE = std::numeric_limits<double>::max();

int main(int argc, char *argv[]) {
	int i, j;

	if (argc < 3) {
		std::cerr << "usage:" << std::endl << argv[0] << " s n k" << std::endl
		          << "Generates a k-nearest connected graph with n vertices in a grid of size s*s." << std::endl;
		return 1;
	}

	const int s = strtol(argv[1], nullptr, 10);
	const int n = strtol(argv[2], nullptr, 10);
	const int k = strtol(argv[3], nullptr, 10);

	if (n > s * s) {
		std::cerr << "n must be less than or equal to s*s." << std::endl;
		return 1;
	}

	if (k >= n) {
		std::cerr << "k must be less than n." << std::endl;
		return 1;
	}

	// init random distribution
	std::random_device dev;
	std::mt19937 rng(dev());
	std::uniform_int_distribution<std::mt19937::result_type> dist(1, s - 1);

	// generate n random unique points
	std::cerr << "Generating random vertexes..." << std::endl;
	std::set<Vertex> vertex_set;

	i = n;
	while (i > 0) {
		Vertex v(dist(rng), dist(rng));

		if (!vertex_set.contains(v)) {
			vertex_set.insert(v);
			i--;
		}
	}

	std::vector<Vertex> vertex_vector(vertex_set.begin(), vertex_set.end());

	// calculate weights
	std::cerr << "Calculating weights..." << std::endl;
	auto **weights = new double *[n];

	for (i = 0; i < n; i++) {
		weights[i] = new double[n];
		for (j = 0; j < n; j++) {
			if (i == j) {
				weights[i][j] = MAX_DOUBLE;
			} else {
				Vertex a = vertex_vector[i];
				Vertex b = vertex_vector[j];
				weights[i][j] = sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
			}
		}
	}

    std::ostringstream graph_filename;
    graph_filename << "k-neargraph_" << s << "_" << n << "_" << k << "_";

    int filename_num = 1;
    FILE *graph_file;

    while ((graph_file = fopen((graph_filename.str() + std::to_string(filename_num) + ".txt").c_str(), "r")) != nullptr)
        filename_num++;
    fclose(graph_file);

    graph_file = fopen((graph_filename.str() + std::to_string(filename_num) + ".txt").c_str(), "w");

	// print graph to stdout
	fprintf(graph_file, "%d\n", n); // print number of vertices
	for (i = 0; i < n; i++) {
		fprintf(graph_file, "%d %d\n", vertex_vector[i].first, vertex_vector[i].second); // print vertex coordinates
	}
	// for each row in weights print k lowest values
	std::cerr << "Looking for k-neighbors..." << std::endl;
	for (i = 0; i < n; i++) {
		for (j = 0; j < k; j++) {
			int min = 0;
			for (int ii = 0; ii < n; ii++) {
				if (weights[i][ii] < weights[i][min]) {
					min = ii;
				}
			}
			fprintf(graph_file, "%d %d %lf\n", i, min, weights[i][min]); // print edge (first vertex, second vertex, weight)
			weights[i][min] = MAX_DOUBLE;
		}
	}

	// TODO check if graph is connected

    fclose(graph_file);
	std::cerr << "Done!" << std::endl;

	return 0;
}
