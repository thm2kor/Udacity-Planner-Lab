#include <iostream>
#include <BehaviorPlanner/vehicle.h>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <cassert>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  state = "CS";
  max_acceleration = -1;
}

Vehicle::~Vehicle() {}

const Vehicle::Pose Vehicle::GetPose(int at_time) const {
  auto temp = state_at(at_time);
  return {temp[0], temp[1], temp[2], temp[3]};
}

void Vehicle::SetPose(Vehicle::Pose pose) {
  lane = pose.lane;
  s = pose.s;
  v = pose.v;
  a = pose.a;
}

// TODO - Implement this method.
void Vehicle::update_state(map<int, Vehicle::Trajectory> predictions) {
  /*
Updates the "state" of the vehicle by assigning one of the
following values to 'self.state':

"KL" - Keep Lane
- The vehicle will attempt to drive its target speed, unless there is
 traffic in front of it, in which case it will slow down.

"LCL" or "LCR" - Lane Change Left / Right
- The vehicle will IMMEDIATELY change lanes and then follow longitudinal
 behavior for the "KL" state in the new lane.

"PLCL" or "PLCR" - Prepare for Lane Change Left / Right
- The vehicle will find the nearest vehicle in the adjacent lane which is
 BEHIND itself and will adjust speed to try to get behind that vehicle.

INPUTS
- predictions
A dictionary. The keys are ids of other vehicles and the values are arrays
where each entry corresponds to the vehicle's predicted location at the
corresponding timestep. The FIRST element in the array gives the vehicle's
current position. Example (showing a car with id 3 moving at 2 m/s):

{
3 : [
  {"s" : 4, "lane": 0},
  {"s" : 6, "lane": 0},
  {"s" : 8, "lane": 0},
  {"s" : 10, "lane": 0},
]
}

*/
  // this->state = "KL";  // this is an example of how you change state.
  static double CostMax = 100;
  vector<string> possible_states{"KL"};
  if (this->lane != 3) {
    possible_states.push_back("PLCL");
    possible_states.push_back("LCL");
  }
  if (this->lane != 0) {
    possible_states.push_back("PLCR");
    possible_states.push_back("LCR");
  }
  vector<double> cost(possible_states.size(), CostMax);
  int i = 0;
  for (string state : possible_states) {
    printf("%s\t", state.c_str());
    auto pose = GetPose();
    if (state == "LCL" && this->lane != 3) {
      pose.lane += 1;
      state = "KL";
    }
    if (state == "LCR" && this->lane != 0) {
      pose.lane -= 1;
      state = "KL";
    }
    auto cost1 = CollisionCost(state, pose, predictions) * collision_w / 1000.;
    auto cost2 =
        LaneChangeCost(state, pose, predictions) * lanechange_w / 1000.;
    cost[i] = cost1;
    cost[i] += cost2;
    printf("\tcost_c %.3f cost_l %.3f cost= %.3f\n", cost1, cost2, cost[i]);
    i++;
  }
  double min_cost = CostMax;
  string best_state = "KL";
  for (int i = 0; i < possible_states.size(); i++) {
    if (cost[i] < min_cost) {
      min_cost = cost[i];
      best_state = possible_states[i];
    }
  }
  this->state = best_state;
  assert(this->lane <= 3);
}

Vehicle::Trajectory Vehicle::GenerateTrajectory(
    map<int, Vehicle::Trajectory> predictions, string state, Pose pose,
    int horizon) const {
  Vehicle veh = *this;
  veh.SetPose(pose);
  Trajectory traj;
  traj.reserve(horizon);
  traj.push_back({veh.lane, veh.s, veh.v, veh.a});
  for (int i = 0; i < horizon; i++) {
    // pretend to be in new proposed state
    veh.state = state;
    veh.realize_state(predictions);
    veh.increment(1);
    traj.push_back({veh.lane, veh.s, veh.v, veh.a});
    for (auto& pred : predictions) {
      pred.second.erase(pred.second.begin());
    }
  }

  return traj;
}

double Vehicle::CollisionCost(
    const string& state, const Vehicle::Pose& pose,
    const map<int, Vehicle::Trajectory>& predictions) const {
  auto traj = GenerateTrajectory(predictions, state, pose);
  auto cars_in_lane = FilterPrediction(predictions, pose.lane);
  Vehicle::collider collider{false, 999, 999};
  Vehicle veh = *this;
  veh.lane = pose.lane;
  for (auto& car : cars_in_lane) {
    printf("{#%d: ", car.first);
    auto temp = veh.will_collide_with(car.second, 10);
    printf("} ");
    if (temp.collision) {
      collider.collision |= temp.collision;
      collider.time = std::min(collider.time, temp.time);
      collider.s = std::min(collider.s, temp.s);
    }
  }

  double delta_s = 0;
  if (collider.collision) {
    delta_s = collider.s - pose.s;
  } else {
    delta_s = traj.back().s - pose.s;
  }

  double ratio = 10;
  double temp = -(fabs(delta_s) / ratio);
  temp = std::min(0.0, temp);
  // printf("\t d_s=%.1f", temp);
  return exp(temp);
}

double Vehicle::LaneChangeCost(
    const string& state, const Vehicle::Pose& pose,
    const map<int, Vehicle::Trajectory>& predictions) const {
  double delta_d = abs(goal_lane - pose.lane) * 6;
  double delta_s = abs(goal_s - pose.s);
  delta_d = max(delta_d, 1e-1);
  delta_s = max(delta_s, 1e-5);
  return 1 - exp(-delta_d / delta_s);
}

void Vehicle::configure(vector<int> road_data) {
  /*
Called by simulator before simulation begins. Sets various
parameters which will impact the ego vehicle.
*/
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
  collision_w = road_data[5];
  lanechange_w = road_data[6];
}

string Vehicle::display() {
  ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->v << "\n";
  oss << "a:    " << this->a << "\n";

  return oss.str();
}

void Vehicle::increment(int dt = 1) {
  this->s += this->v * dt;
  this->v += this->a * dt;
}

// Predicts state of vehicle in t seconds (assuming constant acceleration)
// vector<int> Vehicle::state_at(int t) const {
//  int s = this->s + this->v * t + this->a * t * t / 2;
//  int v = this->v + this->a * t;
//  return {this->lane, s, v, this->a};
//}

// Predicts state of vehicle in t seconds (assuming constant velocity)
vector<int> Vehicle::state_at(int t) const {
  int s = this->s + this->v * t;
  int v = this->v;
  return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Pose other_pose, int at_time) {
  auto self_pose = GetPose(at_time);
  return (self_pose.lane == other_pose.lane) &&
         (abs(self_pose.s - other_pose.s) <= L);
}

bool Vehicle::collides_with(Vehicle::Pose other_pose, int at_time,
                            int at_time_next) {
  auto self_pose1 = GetPose(at_time);
  auto self_pose2 = GetPose(at_time_next);
  if (self_pose1.lane != other_pose.lane) return false;
  // printf("[%d->%d # %d]", self_pose1.s, self_pose2.s, other_pose.s);
  return (self_pose1.s - other_pose.s) * (self_pose2.s - other_pose.s) < 0;
}

Vehicle::collider Vehicle::will_collide_with(Vehicle::Trajectory other,
                                             int timesteps) {
  Vehicle::collider collider_temp{false, -999, -999};

  for (int t = 0; t < timesteps + 1; t++) {
    if (collides_with(other[t], t)) {
      collider_temp.collision = true;
      collider_temp.time = t;
      collider_temp.s = other[t].s;
      printf("[%d:%d X %d] ", GetPose(t).lane, GetPose(t).s, other[t].s);
      printf("X@ %d ", collider_temp.s);
      return collider_temp;
    } else if (collides_with(other[t], t, t + 1)) {
      collider_temp.collision = true;
      collider_temp.time = t;
      collider_temp.s = other[t].s;
      printf("[%d:%d X %d] ", GetPose(t).lane, GetPose(t).s, other[t].s);
      printf("XX@ %d ", collider_temp.s);
      return collider_temp;
    }
  }

  // printf("\n");
  return collider_temp;
}

void Vehicle::realize_state(map<int, Vehicle::Trajectory> predictions) {
  /*
Given a state, realize it by adjusting acceleration and lane.
Note - lane changes happen instantaneously.
*/
  string state = this->state;
  if (state.compare("CS") == 0) {
    realize_constant_speed();
  } else if (state.compare("KL") == 0) {
    realize_keep_lane(predictions);
  } else if (state.compare("LCL") == 0) {
    realize_lane_change(predictions, "L");
  } else if (state.compare("LCR") == 0) {
    realize_lane_change(predictions, "R");
  } else if (state.compare("PLCL") == 0) {
    realize_prep_lane_change(predictions, "L");
  } else if (state.compare("PLCR") == 0) {
    realize_prep_lane_change(predictions, "R");
  }
}

void Vehicle::realize_constant_speed() { a = 0; }

int Vehicle::_max_accel_for_lane(map<int, Vehicle::Trajectory> predictions,
                                 int lane, int s) {
  int delta_v_til_target = target_speed - v;
  int max_acc = min(max_acceleration, delta_v_til_target);

  map<int, Vehicle::Trajectory>::iterator it = predictions.begin();
  vector<Vehicle::Trajectory> in_front;
  while (it != predictions.end()) {
    int v_id = it->first;

    Vehicle::Trajectory v = it->second;

    if ((v[0].lane == lane) && (v[0].s > s)) {
      in_front.push_back(v);
    }
    it++;
  }

  if (in_front.size() > 0) {
    int min_s = 1000;
    Vehicle::Trajectory leading = {};
    for (int i = 0; i < in_front.size(); i++) {
      if ((in_front[i][0].s - s) < min_s) {
        min_s = (in_front[i][0].s - s);
        leading = in_front[i];
      }
    }

    int next_pos = leading[1].s;
    int my_next = s + this->v;
    int separation_next = next_pos - my_next;
    int available_room = separation_next - preferred_buffer;
    max_acc = min(max_acc, available_room);
  }

  return max_acc;
}

void Vehicle::realize_keep_lane(map<int, Vehicle::Trajectory> predictions) {
  this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int, Vehicle::Trajectory> predictions,
                                  string direction) {
  int delta = -1;
  if (direction.compare("L") == 0) {
    delta = 1;
  }
  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(
    map<int, Vehicle::Trajectory> predictions, string direction) {
  int delta = -1;
  if (direction.compare("L") == 0) {
    delta = 1;
  }
  int lane = this->lane + delta;

  map<int, Vehicle::Trajectory>::iterator it = predictions.begin();
  vector<Vehicle::Trajectory> at_behind;
  while (it != predictions.end()) {
    int v_id = it->first;
    Vehicle::Trajectory v = it->second;

    if ((v[0].lane == lane) && (v[0].s <= this->s)) {
      at_behind.push_back(v);
    }
    it++;
  }
  if (at_behind.size() > 0) {
    int max_s = -1000;
    Vehicle::Trajectory nearest_behind = {};
    for (int i = 0; i < at_behind.size(); i++) {
      if ((at_behind[i][0].s) > max_s) {
        max_s = at_behind[i][0].s;
        nearest_behind = at_behind[i];
      }
    }
    int target_vel = nearest_behind[1].s - nearest_behind[0].s;
    int delta_v = this->v - target_vel;
    int delta_s = this->s - nearest_behind[0].s;
    if (delta_v != 0) {
      int time = -2 * delta_s / delta_v;
      int a;
      if (time == 0) {
        a = this->a;
      } else {
        a = delta_v / time;
      }
      if (a > this->max_acceleration) {
        a = this->max_acceleration;
      }
      if (a < -this->max_acceleration) {
        a = -this->max_acceleration;
      }
      this->a = a;
    } else {
      int my_min_acc = max(-this->max_acceleration, -delta_s);
      this->a = my_min_acc;
    }
  }
}

Vehicle::Trajectory Vehicle::generate_predictions(int horizon = 9) {
  Vehicle::Trajectory predictions;
  for (int i = 0; i < horizon; i++) {
    predictions.push_back(GetPose(i));
  }
  return predictions;
}

map<int, Vehicle::Trajectory> Vehicle::FilterPrediction(
    const map<int, Vehicle::Trajectory>& predictions, const int lane) const {
  map<int, Vehicle::Trajectory> vehs;
  for (auto& pred : predictions) {
    if (lane == pred.second[0].lane && pred.first != -1) {
      vehs[pred.first] = pred.second;
    }
  }
  return vehs;
}
