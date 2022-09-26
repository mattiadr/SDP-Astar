#include <queue>
#include <barrier>
#include <boost/lockfree/queue.hpp>

#include "../include/graph_utils/graph_utils.h"

#define N_THREADS 16
#define FREELIST_SIZE 1024

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

const auto queue_comparator = [](NodeFCost a, NodeFCost b) { return a.second > b.second; };

std::vector<std::thread> threads(N_THREADS);
std::vector<std::unique_ptr<lockfree::queue<Message>>> messageQueues(N_THREADS);
std::barrier barrier(N_THREADS);
std::vector<bool> finished(N_THREADS);
std::vector<NodeId> path;

/** functions **/

void broadcast_message(Message m, unsigned int senderId) {
	for (int i = 0; i < N_THREADS; i++) {
		if (i == senderId) continue;
		messageQueues[i]->push(m);
	}
}

bool has_finished() {
	for (int i = 0; i < N_THREADS; i++) {
		if (!finished[i]) {
			return false;
		}
	}
	return true;
}

void
hdastar_distributed(unsigned int threadId, const Graph &g, NodeId pathStart, NodeId pathEnd) {
	std::priority_queue<NodeFCost, std::vector<NodeFCost>, decltype(queue_comparator)> openSet(queue_comparator);
	std::unordered_map<NodeId, double> costToCome;
	std::unordered_map<NodeId, double>::iterator iter;
	std::unordered_map<NodeId, NodeId> cameFrom;
	Message m;

	while (true) {
		// empty message queue
		while (messageQueues[threadId]->pop(m)) {
			switch (m.type) {
				// move work messages to open set if no duplicates
				case WORK:
					iter = costToCome.find(m.target);
					if (iter == costToCome.end() || iter->second > m.gCost) {
						openSet.push(NodeFCost(m.target, m.fCost));
						costToCome.insert_or_assign(m.target, m.gCost);
						cameFrom.insert_or_assign(m.target, m.parent);
					}
					break;

				case TARGET_REACHED:
					// TODO
					std::cout << "Target Reached!" << std::endl;
					break;
				default:
					std::cerr << "WORK " << threadId << " : Invalid message type: " << m.type << std::endl;
			}
		}

		// termination condition
		if (openSet.empty()) {
			barrier.arrive_and_wait();
			// set finished flag
			finished[threadId] = messageQueues[threadId]->empty();
			barrier.arrive_and_wait();

			if (threadId == 0) {
				for (int i = 0; i < N_THREADS; i++) {
					std::cout << finished[i] << ", ";
				}
				std::cout << std::endl;
			}
			// check if all threads finished working, otherwise continue
			if (has_finished()) {
				break;
			} else {
				continue;
			}
		}

		// pop first from open set
		NodeFCost n = openSet.top();
		openSet.pop();

		// check if we reached end of path
		if (n.first == pathEnd) {
			// TODO broadcast TARGET_REACHED for pruning
			continue;
		}

		// iterate over neighbors
		double ctc = costToCome.at(n.first);
		for (auto neighbor: make_iterator_range(out_edges(n.first, g))) {
			double weight = get(edge_weight, g, neighbor);
			double gCost = ctc + weight;
			double fCost = gCost + calc_h_cost(g, neighbor.m_target, pathEnd);

			unsigned int targetThread = hash_node_id(neighbor.m_target, N_THREADS);
			if (targetThread == threadId) {
				// send to this open set
				iter = costToCome.find((NodeId) neighbor.m_target);
				if (iter == costToCome.end() || iter->second > gCost) {
					openSet.push(NodeFCost((NodeId) neighbor.m_target, fCost));
					costToCome.insert_or_assign((NodeId) neighbor.m_target, gCost);
					cameFrom.insert_or_assign((NodeId) neighbor.m_target, n.first);
				}
			} else {
				// create message
				Message outgoing{WORK, (NodeId) neighbor.m_target, n.first, fCost, gCost};
				// send message
				messageQueues[targetThread]->push(outgoing);
			}
		}
	}

	// Path Reconstruction
	if (hash_node_id(pathEnd, N_THREADS) == threadId) {
		messageQueues[threadId]->push(Message{PATH_RECONSTRUCTION, pathEnd});
	}

	NodeId prev;
	unsigned int prevThread;

	while (true) {
		if (!messageQueues[threadId]->pop(m)) {
			continue;
		}

		switch (m.type) {
			case PATH_RECONSTRUCTION:
				path.insert(path.begin(), m.target);

				if (m.target == pathStart) {
					broadcast_message(Message{PATH_END}, threadId);
					return;
				} else {
					prev = cameFrom.at(m.target);
					prevThread = hash_node_id(prev, N_THREADS);
					messageQueues[prevThread]->push(Message{PATH_RECONSTRUCTION, prev});
				}
				break;
			case PATH_END:
				return;
			default:
				std::cerr << "PATH RECONSTRUCTION " << threadId << " : Invalid message type: " << m.type << std::endl;
		}
	}
}

int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " FILENAME" << std::endl;
		return 1;
	}
	char* filename = argv[1];
	Graph g = read_graph(filename);
	unsigned int N = num_vertices(g);

	for (int i = 0; i < N_THREADS; i++) {
		messageQueues[i] = std::make_unique<lockfree::queue<Message>>(FREELIST_SIZE);
	}

	Message m{WORK, (NodeId) 0, (NodeId) 0, 0, 0};
	messageQueues[hash_node_id(0, N_THREADS)]->push(m);

	for (int i = 0; i < N_THREADS; i++) {
		threads[i] = std::thread(hdastar_distributed, i, g, 0, N - 1);
	}

	for (int i = 0; i < N_THREADS; i++) {
		threads[i].join();
	}

	for (unsigned int node : path) {
		std::cout << node << " -> ";
	}
	std::cout << std::endl;
}
