#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
 public:
  struct collider {
    bool collision;  // is there a collision?
    int time;        // time collision happens
    int s;           // distance collision happens
  };

  struct Pose {
    int lane;
    int s;
    int v;
    int a;
  };

  typedef vector<Pose> Trajectory;

  int L = 1;

  int preferred_buffer = 6;  // impacts "keep lane" behavior.

  int lane;

  int s;

  int v;

  int a;

  int target_speed;

  int lanes_available;

  int max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  int lanechange_w = 0;

  int collision_w = 0;

  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  const Pose GetPose(int at_time = 0) const;

  void SetPose(Pose pose);

  void update_state(map<int, Trajectory> predictions);

  void configure(vector<int> road_data);

  string display();

  void increment(int dt);

  vector<int> state_at(int t) const;

  bool collides_with(Pose other_pose, int at_time);

  bool collides_with(Pose other_pose, int at_time, int at_time_next);

  collider will_collide_with(Trajectory other, int timesteps);

  void realize_state(map<int, Trajectory> predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(map<int, Trajectory> predictions, int lane, int s);

  void realize_keep_lane(map<int, Trajectory> predictions);

  void realize_lane_change(map<int, Trajectory> predictions, string direction);

  void realize_prep_lane_change(map<int, Trajectory> predictions,
                                string direction);

  Trajectory generate_predictions(int horizon);

  map<int, Trajectory> FilterPrediction(const map<int, Trajectory>& predictions,
                                        const int lane) const;

  Trajectory GenerateTrajectory(map<int, Trajectory> predictions, string state,
                                Pose pose, int horizon = 9) const;

  double CollisionCost(const string& state, const Pose& pose,
                       const map<int, Trajectory>& predictions) const;

  double LaneChangeCost(const string& state, const Pose& pose,
                        const map<int, Trajectory>& predictions) const;
};

#endif
