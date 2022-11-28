#include <queue>
#include <barrier>
#include <cfloat>
#include <thread>
#include <chrono>
#include <semaphore>
#include <boost/lockfree/queue.hpp>

#include "../include/graph_utils/graph_utils.h"
#include "../include/stats/stats.h"

#define N_THREADS 16
#define FREELIST_SIZE 32

using namespace boost;


/** types **/

typedef std::pair<NodeId, double> NodeFCost;

typedef enum {
	WORK,
	TARGET_REACHED,
	PATH_RECONSTRUCTION,
	PATH_END
} MessageType;

typedef struct {
	MessageType type;
	NodeId target;
	NodeId parent;
	double fCost;
	double gCost;
} Message;


/** globals **/

const auto queue_comparator = [](const NodeFCost &a, const NodeFCost &b) { return a.second > b.second; };

bool PATH_EXISTS = false;
std::vector<std::thread> threads(N_THREADS);
std::vector<std::unique_ptr<lockfree::queue<Message>>> messageQueues(N_THREADS);
std::barrier barrier(N_THREADS);
std::vector<bool> finished(N_THREADS);
bool pathReconstructed = false;
std::vector<NodeId> path;
std::vector<std::unique_ptr<std::counting_semaphore<N_THREADS>>> semaphores(N_THREADS);

/** functions **/

void broadcast_message(const Message &m, const unsigned int senderId) {
	for (int i = 0; i < N_THREADS; i++) {
		if (i == senderId) continue;
		messageQueues[i]->push(m);
	}
}

bool has_finished(double &bestPathWeight) {
	for (int i = 0; i < N_THREADS; i++) {
		if (!finished[i]) {
			return false;
		}
	}
	if (PATH_EXISTS && bestPathWeight == DBL_MAX)
		return false;
	return true;
}

// empty message queue
void process_queue(const unsigned int threadId, Message &m,
                   std::priority_queue<NodeFCost, std::vector<NodeFCost>, decltype(queue_comparator)> &openSet,
                   double *costToCome, NodeId *cameFrom, double &bestPathWeight) {
	while (messageQueues[threadId]->pop(m)) {
		switch (m.type) {
			// move work messages to open set if no duplicates
			case WORK:
				if (m.fCost < bestPathWeight) {
					if (costToCome[m.target] > m.gCost) {
						openSet.push(NodeFCost(m.target, m.fCost));
						costToCome[m.target] = m.gCost;
						cameFrom[m.target] = m.parent;
					}
				}
				break;

			case TARGET_REACHED:
				if (m.fCost < bestPathWeight)
					bestPathWeight = m.fCost;
				break;
			default:
				std::cerr << "WORK thread " << threadId << " : Invalid message type: " << m.type << std::endl;
		}
	}
}

void hdastar_distributed(const unsigned int threadId, const Graph &g, const NodeId &pathStart, const NodeId &pathEnd,
                         stats &stat) {
	std::priority_queue<NodeFCost, std::vector<NodeFCost>, decltype(queue_comparator)> openSet(queue_comparator);
	unsigned int N = num_vertices(g);
	double *costToCome = new double[N];
	std::fill_n(costToCome, N, DBL_MAX);
	NodeId *cameFrom = new NodeId[N];
	std::fill_n(cameFrom, N, INVALID_NODE_ID);
	double bestPathWeight = DBL_MAX;
	Message m;

	while (true) {
		// empty message queue
		process_queue(threadId, m, openSet, costToCome, cameFrom, bestPathWeight);

		// termination condition
		if (openSet.empty()) {
			barrier.arrive_and_wait();
			// set finished flag
			process_queue(threadId, m, openSet, costToCome, cameFrom, bestPathWeight);
			finished[threadId] = openSet.empty();
			barrier.arrive_and_wait();

			// check if all threads finished working, otherwise continue
			if (has_finished(bestPathWeight)) {
				break;
			} else {
				continue;
			}
		}

		// pop first from open set
		NodeFCost n = openSet.top();
		openSet.pop();
		if (n.second >= bestPathWeight)
			continue;

		// check if we reached end of path
		if (n.first == pathEnd) {
			// DONE broadcast TARGET_REACHED for pruning
			if (n.second < bestPathWeight) {
				bestPathWeight = n.second;
				Message targetReached{.type = TARGET_REACHED, .target = n.first, .fCost = n.second};
				broadcast_message(targetReached, threadId);
			}
			continue;
		}

		// iterate over neighbors
		double ctc = costToCome[n.first];
		stat.addNodeVisited(threadId);
		for (auto neighbor: make_iterator_range(out_edges(n.first, g))) {
			double weight = get(edge_weight, g, neighbor);
			double gCost = ctc + weight;
			double fCost = gCost + calc_h_cost(g, neighbor.m_target, pathEnd);

			if (fCost < bestPathWeight) {
				unsigned int targetThread = hash_node_id(neighbor.m_target, N_THREADS);
				if (targetThread == threadId) {
					// send to this open set
					if (costToCome[neighbor.m_target] > gCost) {
						openSet.push(NodeFCost(neighbor.m_target, fCost));
						costToCome[neighbor.m_target] = gCost;
						cameFrom[neighbor.m_target] = n.first;
					}
				} else {
					// create message
					Message outgoing{.type = WORK, .target = (NodeId) neighbor.m_target, .parent = n.first, .fCost = fCost, .gCost = gCost};
					// send message
					messageQueues[targetThread]->push(outgoing);
				}
			}
		}
	}

	if (threadId == 0)
		stat.timeStep("Astar");

	delete[] costToCome;

	// Path Reconstruction
	if (hash_node_id(pathEnd, N_THREADS) == threadId) {
		messageQueues[threadId]->push(Message{.type = PATH_RECONSTRUCTION, .target = pathEnd});
		semaphores[threadId]->release();
	}

	NodeId prev;
	unsigned int prevThread;

	while (true) {
		semaphores[threadId]->acquire();
		if (!messageQueues[threadId]->pop(m)) {
			continue;
		}
		switch (m.type) {
			case PATH_RECONSTRUCTION:
				path.insert(path.begin(), m.target);

				if (m.target == pathStart) {
					broadcast_message(Message{.type = PATH_END}, threadId);
					pathReconstructed = true;
					for (int i = 0; i < N_THREADS; i++)
						if (i != threadId)
							semaphores[i]->release();
					delete[] cameFrom;
					return;
				} else {
					prev = cameFrom[m.target];
					if (prev == INVALID_NODE_ID) {
						std::cerr << "hdastar_message_passing: Error reconstructing path: " << m.target
						          << " Nodes parent not found" << std::endl;
						pathReconstructed = false;
						broadcast_message(Message{.type = PATH_END}, threadId);
						for (int i = 0; i < N_THREADS; i++)
							if (i != threadId)
								semaphores[i]->release();
						delete[] cameFrom;
						return;
					}
					prevThread = hash_node_id(prev, N_THREADS);
					messageQueues[prevThread]->push(Message{.type = PATH_RECONSTRUCTION, .target = prev});
					semaphores[prevThread]->release();
				}
				break;
			case PATH_END:
				delete[] cameFrom;
				return;
			default:
				std::cerr << "PATH RECONSTRUCTION " << threadId << " : Invalid message type: " << m.type << std::endl;
		}
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

	// launcher mode
	if (argc >= 6)
		PATH_EXISTS = true;

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
		stats s("HDA* Message Passing", N_THREADS, filename, seed);
		s.timeStep("Start");

		// init global variables
		for (int j = 0; j < N_THREADS; j++) {
			messageQueues[j] = std::make_unique<lockfree::queue<Message>>(FREELIST_SIZE);
			semaphores[j] = std::make_unique<std::counting_semaphore<N_THREADS>>(0);
		}

		Message m{.type = WORK, .target = (NodeId) source, .parent = (NodeId) source, .fCost = 0, .gCost = 0};
		messageQueues[hash_node_id(source, N_THREADS)]->push(m);
		s.timeStep("Queues init");

		// run threads
		for (int j = 0; j < N_THREADS; j++) {
			threads[j] = std::thread(hdastar_distributed, j, ref(g), source, dest, ref(s));
		}

		for (int j = 0; j < N_THREADS; j++) {
			threads[j].join();
		}

		// print stats on success
		if (pathReconstructed) {
			s.timeStep("Path reconstruction");
			s.printTimeStats();

			// Print paths total cost
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

		// cleanup global variables
		std::fill(finished.begin(), finished.end(), false);
		pathReconstructed = false;
		path.clear();
	}

	return 0;
}
