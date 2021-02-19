#include <BehaviorPlanner/road.h>
#include <BehaviorPlanner/vehicle.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <highgui.h>

void DrawImage(const RoadStringImage& str_img, const Vehicle& ego_car,
               int step) {
  // OpenCV coordinate system:
  // 0 ---- x
  // |
  // |
  // y

  // Road coordinate system:
  // 0 ----lane(0,1,2,3...)
  // |
  // |
  // distance

  cv::Mat image = cv::Mat::zeros(640, 320, CV_8UC1);
  image.setTo(cv::Scalar(255));

  // Draw Headline
  string headline = "Meters / Step: " + to_string(step) + " | " +
                    ego_car.state + " V " + to_string(ego_car.v) + " A " +
                    to_string(ego_car.a);
  cv::Point pos(0, 15);
  cv::putText(image, headline, pos,     // Coordinates
              cv::FONT_HERSHEY_DUPLEX,  // Font
              0.5,                      // Scale. 2.0 = 2x bigger
              cv::Scalar(0),            // Color
              1);                       // Thickness

  static double lane_ratio = 60;
  static double col_ratio = 15;
  // Draw Lane lines
  cv::Point offset(60, 25);
  for (auto lane : {0, 1, 2, 3, 4}) {
    cv::Point start(lane * lane_ratio, 0);
    cv::Point end(lane * lane_ratio, 600);
    cv::line(image, start + offset, end + offset, cv::Scalar(0));
  }

  // Draw car IDs
  auto& road = str_img.road;
  for (int col = 0; col < road.size(); col++) {
    for (int lane = 0; lane < road[col].size(); lane++) {
      cv::Point pos(lane * lane_ratio + lane_ratio / 8, col * col_ratio + 16);
      cv::putText(image, road[col][lane], pos + offset,  // Coordinates
                  cv::FONT_HERSHEY_DUPLEX,               // Font
                  0.5,            // Scale. 2.0 = 2x bigger
                  cv::Scalar(0),  // Color
                  1);             // Thickness
    }
  }

  // Draw distance
  auto& dist = str_img.distance;
  for (auto& s : dist) {
    cv::Point pos(5, s.first * col_ratio + 16 + offset.y);
    cv::putText(image, s.second, pos,     // Coordinates
                cv::FONT_HERSHEY_DUPLEX,  // Font
                0.5,                      // Scale. 2.0 = 2x bigger
                cv::Scalar(0),            // Color
                1);                       // Thickness
  }

  char buffer[10];
  sprintf(buffer, "%03d", step);
  cv::imwrite("images/step_" + string(buffer) + ".jpg", image);
}

using namespace std;

// impacts default behavior for most states
int SPEED_LIMIT = 10;

// all traffic in lane (besides ego) follow these speeds
vector<int> LANE_SPEEDS = {6, 7, 8, 9};

// Number of available "cells" which should have traffic
double TRAFFIC_DENSITY = 0.15;

// At each timestep, ego can set acceleration to value between
// -MAX_ACCEL and MAX_ACCEL
int MAX_ACCEL = 2;

// s value and lane number of goal.
vector<int> GOAL = {300, 0};

// These affect the visualization
int FRAMES_PER_SECOND = 4;
int AMOUNT_OF_ROAD_VISIBLE = 40;

void ReadWeights(string file_name, int& collision_w, int& lanechange_w) {
  fstream in(file_name);
  // The first line in the file is comment
  in.ignore(256, '\n');

  string dummy;
  in >> dummy >> lanechange_w;
  in >> dummy >> collision_w;
  printf("lanchange weight %d collision weight %d\n", lanechange_w,
         collision_w);
}

int main(int argc, char** argv) {
  int seed = 0;
  if (argc == 2) {
    seed = std::atoi(argv[1]);
  }
  Road road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS, seed);
  road.SetGoal(GOAL[0], GOAL[1]);

  road.update_width = AMOUNT_OF_ROAD_VISIBLE;

  road.populate_traffic();

  int goal_s = 300;
  int goal_lane = 0;

  // configuration data: speed limit, num_lanes, goal_s, goal_lane,
  // max_acceleration

  int collision_w = 0, lanechange_w = 0;
  ReadWeights("../weights.data", collision_w, lanechange_w);
  int num_lanes = LANE_SPEEDS.size();
  vector<int> ego_config = {SPEED_LIMIT, num_lanes,   goal_s,      goal_lane,
                            MAX_ACCEL,   collision_w, lanechange_w};

  road.add_ego(2, 0, ego_config);
  int timestep = 0;

  // Create a folder to save test images
  system("rm -rf images");
  system("mkdir images");

  auto str_img = road.display(0);
  DrawImage(str_img, road.get_ego(), 0);
  while (road.get_ego().s <= GOAL[0]) {
    printf("Ego s %d lane %d\n", road.get_ego().s, road.get_ego().lane);
    timestep++;
    if (timestep > 35) {
      break;
    }
    road.advance();
    auto str_img = road.display(timestep);
    DrawImage(str_img, road.get_ego(), timestep);
    // time.sleep(float(1.0) / FRAMES_PER_SECOND);
  }
  Vehicle ego = road.get_ego();
  if (ego.lane == GOAL[1]) {
    cout << "You got to the goal in " << timestep << " seconds!" << endl;
    if (timestep > 35) {
      cout << "But it took too long to reach the goal. Go faster!" << endl;
    }
  } else {
    cout << "You missed the goal. You are in lane " << ego.lane
         << " instead of " << GOAL[1] << "." << endl;
  }
  return 0;
}
