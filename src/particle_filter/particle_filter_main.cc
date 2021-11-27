//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter-main.cc
\brief   Main entry point for particle filter based
         mobile robot localization
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <termios.h>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"

#include "config_reader/config_reader.h"
#include "shared/math/math_util.h"
#include "shared/math/line2d.h"
#include "shared/util/timer.h"

#include "particle_filter.h"
#include "visualization/visualization.h"

using amrl_msgs::VisualizationMsg;
using geometry::line2f;
using geometry::Line;
using math_util::DegToRad;
using math_util::RadToDeg;
using ros::Time;
using std::string;
using std::vector;
using Eigen::Vector2f;
using visualization::ClearVisualizationMsg;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::DrawLine;
using visualization::DrawParticle;
using visualization::DrawCross;
using visualization::DrawLine;

// Create command line arguements
DEFINE_string(laser_topic, "/scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "/odom", "Name of ROS topic for odometry data");
DEFINE_string(init_topic,
              "/set_pose",
              "Name of ROS topic for initialization");

DECLARE_int32(v);

// Create config reader entries
CONFIG_STRING(map_name_, "map");
CONFIG_FLOAT(init_x_, "init_x");
CONFIG_FLOAT(init_y_, "init_y");
CONFIG_FLOAT(init_r_, "init_r");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

bool run_ = true;
particle_filter::ParticleFilter particle_filter_;
ros::Publisher visualization_publisher_;
ros::Publisher localization_publisher_;
ros::Publisher laser_publisher_;
VisualizationMsg vis_msg_;
sensor_msgs::LaserScan last_laser_msg_;

vector<Vector2f> trajectory_points_;

void InitializeMsgs() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;

  vis_msg_ = visualization::NewVisualizationMessage("map", "particle_filter");
}

void PublishParticles() {
  vector<particle_filter::Particle> particles;
  particle_filter_.GetParticles(&particles);
  float s = 0;
  
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  for (const particle_filter::Particle& p : particles) {
    uint32_t kColor = 0;
    uint32_t scaledColor = (uint32_t) (int) (((float) 0x0FF) * (1 - p.weight));
    if (scaledColor > 0xFF) {
      scaledColor = 0xFF;
    }
    kColor = scaledColor + (scaledColor << 8) + (scaledColor << 16);
    
    // kColor = 0;
    // if (p.weight > (float) (1/ (float) particles.size())) {
      // printf("good %f %x\n", p.weight, kColor);
    // }
    DrawLine(p.loc, robot_loc, kColor, vis_msg_);
    // if (true)
    // {
    //   float ang = 0.0;
    //   particle_filter::Particle temp;
    //   temp.loc = p.loc;
    //   for (ang = 0.0; ang <= 2 * M_PI; ang += M_PI/2.0)
    //   {
    //     printf("ANG: %f\n", ang);
    //     temp.angle = ang;
    //     particle_filter_.Update(last_laser_msg_.ranges,
    //                             last_laser_msg_.range_min,
    //                             last_laser_msg_.range_max,
    //                             last_laser_msg_.angle_min,
    //                             last_laser_msg_.angle_max,
    //                             &temp);
    //     printf("TEMP: %f\n", exp(temp.weight) * 10.0);
    //     Vector2f weight_dist = Vector2f((temp.loc.x() * exp(temp.weight) * 100.0) * cos(ang), 
    //                                     (temp.loc.y() * exp(temp.weight) * 100.0) * sin(ang));
    //     DrawLine(p.loc, weight_dist, 0xA269E0, vis_msg_);
    //   }
    // }

    DrawParticle(p.loc, p.angle, vis_msg_);
    s+= p.weight;
  }
  // printf("\n");
}

void PublishPredictedScan() {
  const uint32_t kColor = 0xA269E5;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  vector<Vector2f> predicted_scan;
  particle_filter_.GetPredictedPointCloud(
      robot_loc,
      robot_angle,
      last_laser_msg_.ranges.size(),
      last_laser_msg_.range_min,
      last_laser_msg_.range_max,
      last_laser_msg_.angle_min,
      last_laser_msg_.angle_max,
      &predicted_scan);
  // for (const Vector2f& p : predicted_scan) {
  //   // DrawPoint(particle_filter_.RobotToGlobal(p, robot_loc, robot_angle), kColor, vis_msg_);
  //   DrawPoint(p, kColor, vis_msg_);

  // }

  for (unsigned i = 0; i < predicted_scan.size(); i+=particle_filter_.laser_point_trim) {
    DrawPoint(predicted_scan[i], kColor, vis_msg_);
  }
}

// void PublishMap() {
//   auto map = particle_filter_.map_;
//   const uint32_t kColor = 0xFF73D4;

//   for (size_t i = 0; i < map.lines.size(); ++i) {
//       const line2f map_line = map.lines[i];
//       DrawLine(map_line.p0,
//              map_line.p1,
//              kColor,
//              vis_msg_);
//   }

// }

void PublishTrajectory() {
  const uint32_t kColor = 0xadadad;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  static Vector2f last_loc_(0, 0);
  if (!trajectory_points_.empty() &&
      (last_loc_ - robot_loc).squaredNorm() > Sq(1.5)) {
    trajectory_points_.clear();
  }
  if (trajectory_points_.empty() ||
      (robot_loc - last_loc_).squaredNorm() > 0.25) {
    trajectory_points_.push_back(robot_loc);
    last_loc_ = robot_loc;
  }
  for (size_t i = 0; i + 1 < trajectory_points_.size(); ++i) {
    DrawLine(trajectory_points_[i],
             trajectory_points_[i + 1],
             kColor,
             vis_msg_);
  }
}

void PublishRealScan() {
  // printf("Drawing real scan\n");
  const uint32_t kColor = 0xFF73D4;

  float angle_delta = (last_laser_msg_.angle_max - last_laser_msg_.angle_min) / last_laser_msg_.ranges.size();
  float angle = last_laser_msg_.angle_min;

  for (unsigned index = 0; index < last_laser_msg_.ranges.size(); index++) {
    Vector2f true_point = particle_filter_.LaserScanToPoint(angle, last_laser_msg_.ranges[index]);
    // printf("Point: %f %f\n", true_point.x(), true_point.y());
    DrawPoint(true_point, kColor, vis_msg_);
    angle += angle_delta;
  }
  visualization_publisher_.publish(vis_msg_);
}

void PublishVisualization() {
  static double t_last = 0;
  if (GetMonotonicTime() - t_last < 0.05) {
    // Rate-limit visualization.
    return;
  }
  t_last = GetMonotonicTime();
  vis_msg_.header.stamp = ros::Time::now();
  ClearVisualizationMsg(vis_msg_);

  PublishParticles();
  PublishRealScan();
  PublishPredictedScan();
  PublishTrajectory();
  // PublishMap();
  visualization_publisher_.publish(vis_msg_);
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f\n", msg.header.stamp.toSec());
  }
  last_laser_msg_ = msg;
  particle_filter_.ObserveLaser(
      msg.ranges,
      msg.range_min,
      msg.range_max,
      msg.angle_min,
      msg.angle_max);

  
  PublishVisualization();
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  const Vector2f odom_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float odom_angle =
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  particle_filter_.Predict(odom_loc, odom_angle);
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  amrl_msgs::Localization2DMsg localization_msg;
  localization_msg.pose.x = robot_loc.x();
  localization_msg.pose.y = robot_loc.y();
  localization_msg.pose.theta = robot_angle;
  localization_publisher_.publish(localization_msg);
  PublishVisualization();
}

void InitCallback(const amrl_msgs::Localization2DMsg& msg) {
  const Vector2f init_loc(msg.pose.x, msg.pose.y);
  const float init_angle = msg.pose.theta;
  const string map = "maps/" + msg.map + ".txt";
  printf("Initialize: %s (%f,%f) %f\u00b0\n",
         map.c_str(),
         init_loc.x(),
         init_loc.y(),
         RadToDeg(init_angle));
  particle_filter_.Initialize(map, init_loc, init_angle);
  trajectory_points_.clear();
}

void ProcessLive(ros::NodeHandle* n) {
  ros::Subscriber initial_pose_sub = n->subscribe(
      FLAGS_init_topic.c_str(),
      1,
      InitCallback);
  ros::Subscriber laser_sub = n->subscribe(
      FLAGS_laser_topic.c_str(),
      1,
      LaserCallback);
  ros::Subscriber odom_sub = n->subscribe(
      FLAGS_odom_topic.c_str(),
      1,
      OdometryCallback);
  while (ros::ok() && run_) {
    ros::spinOnce();
    PublishVisualization();
    Sleep(0.01);
  }
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "particle_filter", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  InitializeMsgs();

  visualization_publisher_ =
      n.advertise<VisualizationMsg>("visualization", 1);
  localization_publisher_ =
      n.advertise<amrl_msgs::Localization2DMsg>("localization", 1);
  laser_publisher_ =
      n.advertise<sensor_msgs::LaserScan>("scan", 1);

  ProcessLive(&n);

  return 0;
}
