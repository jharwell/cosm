/**
 * \file kinematics_metrics_avg_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/kin/metrics/kinematics_metrics_avg_csv_sink.hpp"

#include <numeric>

#include "cosm/kin/metrics/kinematics_metrics.hpp"
#include "cosm/kin/metrics/kinematics_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::kin::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
kinematics_metrics_avg_csv_sink::kinematics_metrics_avg_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
kinematics_metrics_avg_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_traveled_homing",
    "int_avg_twist_linear_velocity_homing",
    "int_avg_twist_angular_velocity_homing",
    "int_avg_pose_orientation_homing",
    "int_avg_pose_position_homing",

    "int_avg_traveled_exploring",
    "int_avg_twist_linear_velocity_exploring",
    "int_avg_twist_angular_velocity_exploring",
    "int_avg_pose_orientation_exploring",
    "int_avg_pose_position_exploring",

    "int_avg_traveled_flocking",
    "int_avg_twist_linear_velocity_flocking",
    "int_avg_twist_angular_velocity_flocking",
    "int_avg_pose_orientation_flocking",
    "int_avg_pose_position_flocking",


    "int_avg_traveled_all",
    "int_avg_twist_linear_velocity_all",
    "int_avg_twist_angular_velocity_all",
    "int_avg_pose_orientation_all",
    "int_avg_pose_position_all",

    "cum_avg_traveled_homing",
    "cum_avg_twist_linear_velocity_homing",
    "cum_avg_twist_angular_velocity_homing",
    "cum_avg_pose_orientation_homing",
    "cum_avg_pose_position_homing",

    "cum_avg_traveled_exploring",
    "cum_avg_twist_linear_velocity_exploring",
    "cum_avg_twist_angular_velocity_exploring",
    "cum_avg_pose_orientation_exploring",
    "cum_avg_pose_position_exploring",

    "cum_avg_traveled_flocking",
    "cum_avg_twist_linear_velocity_flocking",
    "cum_avg_twist_angular_velocity_flocking",
    "cum_avg_pose_orientation_flocking",
    "cum_avg_pose_position_flocking",


    "cum_avg_traveled_all",
    "cum_avg_twist_linear_velocity_all",
    "cum_avg_twist_angular_velocity_all",
    "cum_avg_pose_orientation_all",
    "cum_avg_pose_position_all",

    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
kinematics_metrics_avg_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                            const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = dynamic_cast<const kinematics_metrics_data*>(data);

  std::string line;
  for (size_t i = 0; i < context_type::ekMAX; ++i) {
    auto ienum = static_cast<context_type>(i);
    line += build_from_context(ienum, d, t);
  } /* for(i..) */

  return boost::make_optional(line);
} /* csv_line_build() */

std::string kinematics_metrics_avg_csv_sink::build_from_context(
    const context_type& ctx,
    const kinematics_metrics_data* data,
    const rtypes::timestep& t) {
  std::string line;
  auto int_traveled = std::accumulate(std::begin(data->interval()),
                                      std::end(data->interval()),
                                      0.0,
                                      [&](double sum, const auto &b){
                                        return sum + b.traveled[ctx].v();
                                      });
  auto int_lin_vel = std::accumulate(std::begin(data->interval()),
                                     std::end(data->interval()),
                                     rmath::vector3d(),
                                     [&](auto sum, const auto &b){
                                       return sum + b.twist[ctx].linear;
                                     });
  auto int_ang_vel = std::accumulate(std::begin(data->interval()),
                                     std::end(data->interval()),
                                     rmath::vector3d(),
                                     [&](auto sum, const auto &b){
                                       return sum + b.twist[ctx].angular;
                                     });

  auto int_pose_orient = std::accumulate(std::begin(data->interval()),
                                         std::end(data->interval()),
                                         rmath::euler_angles(),
                                         [&](auto sum, const auto &b){
                                           return sum + b.pose[ctx].orientation;
                                         });
  auto int_pose_pos = std::accumulate(std::begin(data->interval()),
                                      std::end(data->interval()),
                                      rmath::vector3d(),
                                      [&](auto sum, const auto &b){
                                        return sum + b.pose[ctx].position;
                                      });
  line += csv_entry_domavg(int_traveled,
                           output_interval().v() * data->n_robots());
  line += csv_entry_domavg(int_lin_vel,
                           output_interval().v() * data->n_robots());
  line += csv_entry_domavg(int_ang_vel,
                           output_interval().v() * data->n_robots());
  line += csv_entry_domavg(int_pose_orient,
                           output_interval().v() * data->n_robots());
  line += csv_entry_domavg(int_pose_pos,
                           output_interval().v() * data->n_robots());

  auto cum_traveled = std::accumulate(std::begin(data->cum()),
                                      std::end(data->cum()),
                                      0.0,
                                      [&](double sum, const auto &b){
                                        return sum + b.traveled[ctx].v();
                                      });
  auto cum_lin_vel = std::accumulate(std::begin(data->cum()),
                                     std::end(data->cum()),
                                     rmath::vector3d(),
                                     [&](auto sum, const auto &b){
                                       return sum + b.twist[ctx].linear;
                                     });
  auto cum_ang_vel = std::accumulate(std::begin(data->cum()),
                                     std::end(data->cum()),
                                     rmath::vector3d(),
                                     [&](auto sum, const auto &b){
                                       return sum + b.twist[ctx].angular;
                                     });

  auto cum_pose_orient = std::accumulate(std::begin(data->cum()),
                                         std::end(data->cum()),
                                         rmath::euler_angles(),
                                         [&](auto sum, const auto &b){
                                           return sum + b.pose[ctx].orientation;
                                         });
  auto cum_pose_pos = std::accumulate(std::begin(data->cum()),
                                      std::end(data->cum()),
                                      rmath::vector3d(),
                                      [&](auto sum, const auto &b){
                                        return sum + b.pose[ctx].position;
                                      });
  line += csv_entry_domavg(cum_traveled, t.v() * data->n_robots());
  line += csv_entry_domavg(cum_lin_vel, t.v() * data->n_robots());
  line += csv_entry_domavg(cum_ang_vel, t.v() * data->n_robots());
  line += csv_entry_domavg(cum_pose_orient, t.v() * data->n_robots());
  line += csv_entry_domavg(cum_pose_pos, t.v() * data->n_robots());

  return line;
} /* build_from_context() */

} /* namespace cosm::kin::metrics */
