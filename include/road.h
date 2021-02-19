#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include <BehaviorPlanner/vehicle.h>

using namespace std;

struct RoadStringImage {
  vector<vector<string>> road;
  vector<pair<int, string>> distance;
};

class Road {
 public:
  int update_width = 70;

  map<string, string> ego_rep;

  int ego_key = -1;

  int num_lanes;

  vector<int> lane_speeds;

  int speed_limit;

  double density;

  int camera_center;

  map<int, Vehicle> vehicles;

  int vehicles_added = 0;

  int goal_lane = 0;
  int goal_s = 0;

  /**
  * Constructor
  */
  Road(int speed_limit, double traffic_density, vector<int> lane_speeds,
       int seed = 0);

  /**
  * Destructor
  */
  virtual ~Road();

  Vehicle get_ego();

  void populate_traffic();

  void advance();

  RoadStringImage display(int timestep);

  void add_ego(int lane_num, int s, vector<int> config_data);

  void cull();

  void SetGoal(const int s, const int lane);
};
