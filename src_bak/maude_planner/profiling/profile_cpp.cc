//
// Profiler for the default C++ A* algorithm
//
// Built with -I/opt/ros/dashing/include/ -L/opt/ros/dashing/lib/ -lnavfn_planner_core options
//

#include <iostream>
#include <cmath>
#include <fstream>
#include <chrono>

using namespace std;

#include "nav2_navfn_planner/navfn.hpp"

namespace nav2_navfn_planner {
	extern int create_nav_plan_astar(unsigned char*, int, int, int*, int*, float*, int);
}


constexpr int SIDE = 384;

int TEST_SUITE[][4] = {
    {161, 194, 162, 194},
    {161, 194, 161, 193},
    {161, 194, 162, 193},
    {161, 194, 168, 194},
    {162, 189, 236, 189},
    // Repeated
    {162, 189, 236, 189},
    {162, 189, 235, 210},
    {162, 189, 212, 237},
    {162, 189, 287, 237},
    {162, 189, 163, 211},
    {162, 189, 199, 161},
    {162, 189, 235, 198},
    {190, 222, 211, 222},
    {189, 211, 211, 189},
    // Not possible
    // {156, 200, 178, 200}
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

void runTest(COSTTYPE * costmap, float * path, size_t index) {

	// Current test case
	int * tcase = TEST_SUITE[index];

	auto start_time = std::chrono::high_resolution_clock::now();
	int length = nav2_navfn_planner::create_nav_plan_astar(costmap, SIDE, SIDE, tcase + 2, tcase, path, SIDE * 4);
	auto end_time = std::chrono::high_resolution_clock::now();

	std::chrono::duration<double> duration = end_time - start_time;
	double distance = calculate_distance(path, length);

	cout << tcase[0] << ";" << tcase[1] << ";" << tcase[2] << ";" << tcase[3]
	     << ";" << duration.count() << ";" << distance << endl;
}

int main() {

	// Read the default costmap from a file
	COSTTYPE * costmap = new COSTTYPE[SIDE * SIDE];

	ifstream costmapfile("../costmap.bin", std::ios::binary);

	if (!costmapfile.is_open()) {
		cerr << "Cannot find costmap.bin file." << endl;
		return 1;
	}

	if (!costmapfile.read((char *) costmap, SIDE * SIDE * sizeof(COSTTYPE)))
	{
		cerr << "Error when reading costmap." << endl;
		return 2;
	}

	costmapfile.close();

	const size_t nrTest = sizeof(TEST_SUITE) / sizeof(TEST_SUITE[0]);
	float * buffer = new float[SIDE * 4];

	for (size_t i = 0; i < nrTest; i++)
		runTest(costmap, buffer, i);

	return 0;
}
