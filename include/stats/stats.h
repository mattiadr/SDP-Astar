#ifndef STATS_H
#define STATS_H

#include <chrono>
#include <iostream>

using namespace std::chrono;

typedef std::pair<high_resolution_clock::time_point, std::string> TimePointPair;

class stats {
	std::vector<TimePointPair> timePoints;

public:
	void timeStep(const std::string &stepName) {
		timePoints.emplace_back(TimePointPair(high_resolution_clock::now(), stepName));
	}

	void printTimeStats() {
		for(int i = 1; i < timePoints.size(); i++) {
			std::cout << timePoints[i].second << ": " << duration_cast<duration<double>>(timePoints[i].first - timePoints[i - 1].first).count() << " seconds." << std::endl;
		}
	}
};

#endif
