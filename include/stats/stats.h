#ifndef STATS_H
#define STATS_H

#include <chrono>
#include <iostream>
#include <fstream>
#include <atomic>
#include <numeric>

#include "../graph_utils/graph_utils.h"

using namespace std::chrono;

typedef std::pair<high_resolution_clock::time_point, std::string> TimePointPair;

class stats {
	std::string algorithm;
	unsigned int nThreads;
	std::string inputFile;
	unsigned long seed;
	double totalCost;
	unsigned int totalSteps;
	std::vector<TimePointPair> timePoints;
	std::vector<unsigned int> nodeVisited;

public:
	stats(const std::string &algorithm, unsigned int nThreads, const std::string &inputFile, unsigned long seed)
			: algorithm(algorithm), nThreads(nThreads), inputFile(inputFile), seed(seed) {
		nodeVisited = std::vector<unsigned int>(nThreads, 0);
	}

	void setTotalCost(double totalCost) {
		stats::totalCost = totalCost;
	}

	void setTotalSteps(unsigned int totalSteps) {
		stats::totalSteps = totalSteps;
	}

	void addNodeVisited(NodeId threadId) {
		nodeVisited[threadId]++;
	}

	void timeStep(const std::string &stepName) {
		timePoints.emplace_back(TimePointPair(high_resolution_clock::now(), stepName));
	}

	void printTimeStats() {
		for(int i = 1; i < timePoints.size(); i++) {
			std::cout << timePoints[i].second << ": " << duration_cast<duration<double>>(timePoints[i].first - timePoints[i - 1].first).count() << " seconds." << std::endl;
		}
	}

	void dump_csv(const std::vector<NodeId> &path) {
		std::fstream outFile("AstarReport.csv", std::fstream::out | std::fstream::app);
		double graphReadTime = 0, astarTime = 0, pathRecTime = 0;
		for (int i = 1; i < timePoints.size(); i++) {
			if (timePoints[i].second == "Read graph")
				graphReadTime = duration_cast<duration<double>>(timePoints[i].first - timePoints[i - 1].first).count();
			else if (timePoints[i].second == "Astar")
				astarTime = duration_cast<duration<double>>(timePoints[i].first - timePoints[i - 1].first).count();
			else if (timePoints[i].second == "Path reconstruction")
				pathRecTime = duration_cast<duration<double>>(timePoints[i].first - timePoints[i - 1].first).count();
		}
		unsigned int totalNodeVisited = std::reduce(nodeVisited.begin(), nodeVisited.end());
		outFile << algorithm << "," << nThreads << "," << inputFile << "," << seed << "," << totalCost << ","
				<< totalSteps << "," << graphReadTime << "," << astarTime << "," << pathRecTime << "," << totalNodeVisited << ",";
		int i;
		for (i = 0; i < path.size() - 1; i++)
			outFile << path[i] << "-";
		outFile << path[i] <<  std::endl;
		outFile.close();
	}
};

#endif
