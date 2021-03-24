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
#include <utility>
#include <type_traits>

#include <unistd.h>	// chdir
#include <libgen.h>	// dirname

using namespace std;

#include "nav2_navfn_planner/navfn.hpp"

using NavFn = nav2_navfn_planner::NavFn;
// The floating-point number type is that of the getLastPathCost result
using fpnumber = decltype(std::declval<NavFn>().getLastPathCost());

struct CostMap {
	COSTTYPE * data;
	int width;
	int height;
};

int create_nav_plan_astar2(COSTTYPE* costmap, int nx, int ny, int* goal, int* start,
                           fpnumber* plan, int nplan, NavFn*& nav) {

	// This is a replica of the ROS's create_nav_plan_astar,
	// but allowing access to the navigation function

	if (nav == nullptr)
		nav = new NavFn(nx, ny);

	if (nav->nx != nx || nav->ny != ny) {  // check for compatibility with previous call
		delete nav;
		nav = new NavFn(nx, ny);
	}

	  nav->setGoal(goal);
	  nav->setStart(start);

	  nav->costarr = costmap;
	  nav->setupNavFn(true);

	  // calculate the nav fn and path
	  nav->priInc = 2 * COST_NEUTRAL;
	  nav->propNavFnAstar(std::max(nx * ny / 20, nx + ny));

	  // path
	  int len = nav->calcPath(nplan);

	  if (len > 0) {
		for (int i = 0; i < len; i++) {
			plan[i * 2] = nav->pathx[i];
			plan[i * 2 + 1] = nav->pathy[i];
		}
	  }

	  return len;
}

double calculate_distance(fpnumber * path, size_t points) {
	if (points == 0)
		return 0.0;

	double s = 0.0;

	fpnumber x = path[0];
	fpnumber y = path[1];

	for (size_t i = 1; i < points; i++) {
		s += sqrt(pow(path[2 * i] - x, 2) + pow(path[2 * i + 1] - y, 2));
		x = path[2 * i];
		y = path[2 * i + 1];
	}

	return s;
}

void runTest(const CostMap &map, fpnumber* path, int* tcase, NavFn*& navfn) {

	auto start_time = std::chrono::high_resolution_clock::now();
	int length = create_nav_plan_astar2(map.data, map.width, map.height, tcase + 2, tcase, path, 4 * map.width, navfn);
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

	cout << "], \"navfn\": [";

	// Write the potential array in the JSON
	const size_t mapSize = map.width * map.height;

	for (size_t i = 0; i < mapSize; i++)
		cout << navfn->potarr[i] << ((i+1 < mapSize) ? ", " : "");

	cout << "]}" << endl;
}

int readTestFile(istream &in) {

	// Read the configuration data from standard input
	//
	// map_width map_height
	// map_path
	// (x0 y0 x y) *

	CostMap map;
	string map_path;

	in >> map.width >> map.height;

	// The next line is the filename (just in case it contain spaces)
	while (isspace(in.peek()))
		in.get();

	getline(in, map_path);

	if (!in) {
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
	fpnumber * buffer = new fpnumber[8 * map.width];
	// Navigation function object
	NavFn* navfn = nullptr;

	// Keep reading cases from the standard input
	while (true) {
		// The initial and goal position coordinates are read
		// from the standard input
		int positions[4];

		in >> positions[0];

		// We finish at the end of file
		if (in.eof()) {
			break;
		}
		// Format errors also interrupt the execution
		else if (in.fail()) {
			cerr << "Error while reading the test cases." << endl;
			return 4;
		}
		// An initial -1 discards the line as a comment
		else if (positions[0] == -1) {
			in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			continue;
		}

		for (size_t i = 1; i < 4; i++)
			in >> positions[i];

		runTest(map, buffer, positions, navfn);
	}

	delete [] buffer;
	delete navfn;

	return 0;
}

int main(int argc, char* argv[]) {
	// The first argument (if any) is seen as input file,
	// but the standard input is used if omitted

	if (argc < 2)
		return readTestFile(cin);

	else if (strcmp(argv[1],  "-v") == 0) {
		cerr << (std::is_same<fpnumber, float>::value ? "float" : "double") << endl;
		return 0;
	}

	ifstream testfile(argv[1]);

	if (!testfile.is_open()) {
		cerr << "Cannot find " << argv[1] << " test file." << endl;
		return 2;
	}

	// The directory where the test file is contained will be
	// the base for its paths (i.e. the path to the map)
	if (chdir(dirname(argv[1])) == -1)
		cerr << "Cannot change to the test directory." << endl;

	return readTestFile(testfile);
}
