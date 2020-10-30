//
// Profiler for the default C++ A* algorithm
//
// Built with -I/opt/ros/dashing/include/ -L/opt/ros/dashing/lib/ -lnavfn_planner_core options
//

#include <iostream>
#include <cmath>
#include <fstream>
#include <chrono>
#include <string>

using namespace std;

#include "nav2_navfn_planner/navfn.hpp"

namespace nav2_navfn_planner {
	extern int create_nav_plan_astar(unsigned char*, int, int, int*, int*, float*, int);
}

struct CostMap {
	COSTTYPE * data;
	int width;
	int height;
};

double calculate_distance(float * path, size_t points) {
	if (points == 0)
		return 0.0;

	double s = 0.0;

	float x = path[0];
	float y = path[1];

	for (size_t i = 1; i < points; i++) {
		s += sqrt(pow(path[2 * i] - x, 2) + pow(path[2 * i + 1] - y, 2));
		x = path[2 * i];
		y = path[2 * i + 1];
	}

	return s;
}

void runTest(const CostMap &map, float * path, int * tcase) {

	auto start_time = std::chrono::high_resolution_clock::now();
	int length = nav2_navfn_planner::create_nav_plan_astar(map.data, map.width, map.height, tcase + 2, tcase, path, 4 * max(map.width, map.height));
	auto end_time = std::chrono::high_resolution_clock::now();

	std::chrono::duration<double> duration = end_time - start_time;
	double distance = calculate_distance(path, length);

	// Print a JSON object
	cout << "{\"initial\": [" << tcase[0] << ", " << tcase[1] << "], "
	     << "\"goal\": [" << tcase[2] << ", " << tcase[3] << "], "
	     << "\"duration\": " << duration.count() << ", "
	     << "\"length\": " << distance << ", "
	     << "\"path\": [";

	for (size_t i = 0; i < length; i++)
		cout << "[" << path[2*i] << ", " << path[2*i+1] << ((i+1 < length) ? "], " : "]");

	cout << "]}" << endl;
}

int main() {

	// Read the configuration data from standard input
	//
	// map_width map_height
	// map_path
	// (x0 y0 x y) *

	CostMap map;
	string map_path;

	cin >> map.width >> map.height;

	// The next line is the filename (just in case it contain spaces)
	while (isspace(cin.peek()))
		cin.get();

	getline(cin, map_path);

	if (!cin) {
		cerr << "Errors while parsing the initial configuration." << endl;
		return 1;
	}

	// Read the default costmap from a file
	map.data = new COSTTYPE[map.width * map.height];

	ifstream costmapfile(map_path, std::ios::binary);

	if (!costmapfile.is_open()) {
		cerr << "Cannot find " << map_path << " file." << endl;
		return 2;
	}

	if (!costmapfile.read((char *) map.data, map.width * map.height * sizeof(COSTTYPE)))
	{
		cerr << "Error while reading the costmap." << endl;
		return 3;
	}

	costmapfile.close();

	// Buffer for the calculated paths
	float * buffer = new float[4 * max(map.width, map.height)];

	// Keep reading cases from the standard input
	while (true) {
		// The initial and goal position coordinates are read
		// from the standard input
		int positions[4];

		cin >> positions[0];

		// We finish at the end of file
		if (cin.eof()) {
			break;
		}
		// Format errors also interrupt the execution
		else if (cin.fail()) {
			cerr << "Error while reading the test cases." << endl;
			return 4;
		}
		// An initial -1 discards the line as a comment
		else if (positions[0] == -1) {
			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			continue;
		}

		for (size_t i = 1; i < 4; i++)
			cin >> positions[i];

		runTest(map, buffer, positions);
	}

	return 0;
}
