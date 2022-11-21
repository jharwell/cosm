/**
 * \file kinematics_metrics_dist_csv_sink.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/kin/metrics/kinematics_metrics_dist_csv_sink.hpp"

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
kinematics_metrics_dist_csv_sink::kinematics_metrics_dist_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
kinematics_metrics_dist_csv_sink::csv_header_cols(const rmetrics::base_data* data) const {
  auto* d = dynamic_cast<const kinematics_metrics_data*>(data);
  auto merged = dflt_csv_header_cols();

  for (size_t i = 0; i < d->n_robots(); ++i) {
    auto cols = std::list<std::string>{
      /* clang-format off */
      "int_traveled_homing_robot_" + rcppsw::to_string(i),
      "int_twist_linear_velocity_homing_robot_" + rcppsw::to_string(i),
      "int_twist_angular_velocity_homing_robot_" + rcppsw::to_string(i),
      "int_pose_orientation_homing_robot_" + rcppsw::to_string(i),
      "int_pose_position_homing_robot_" + rcppsw::to_string(i),

      "int_traveled_exploring_robot_" + rcppsw::to_string(i),
      "int_twist_linear_velocity_exploring_robot_" + rcppsw::to_string(i),
      "int_twist_angular_velocity_exploring_robot_" + rcppsw::to_string(i),
      "int_pose_orientation_exploring_robot_" + rcppsw::to_string(i),
      "int_pose_position_exploring_robot_" + rcppsw::to_string(i),

      "int_traveled_flocking_robot_" + rcppsw::to_string(i),
      "int_twist_linear_velocity_flocking_robot_" + rcppsw::to_string(i),
      "int_twist_angular_velocity_flocking_robot_" + rcppsw::to_string(i),
      "int_pose_orientation_flocking_robot_" + rcppsw::to_string(i),
      "int_pose_position_flocking_robot_" + rcppsw::to_string(i),

      "int_traveled_all_robot_" + rcppsw::to_string(i),
      "int_twist_linear_velocity_all_robot_" + rcppsw::to_string(i),
      "int_twist_angular_velocity_all_robot_" + rcppsw::to_string(i),
      "int_pose_orientation_all_robot_" + rcppsw::to_string(i),
      "int_pose_position_all_robot_" + rcppsw::to_string(i),

      "cum_traveled_homing_robot_" + rcppsw::to_string(i),
      "cum_twist_linear_velocity_homing_robot_" + rcppsw::to_string(i),
      "cum_twist_angular_velocity_homing_robot_" + rcppsw::to_string(i),
      "cum_pose_orientation_homing_robot_" + rcppsw::to_string(i),
      "cum_pose_position_homing_robot_" + rcppsw::to_string(i),

      "cum_traveled_exploring_robot_" + rcppsw::to_string(i),
      "cum_twist_linear_velocity_exploring_robot_" + rcppsw::to_string(i),
      "cum_twist_angular_velocity_exploring_robot_" + rcppsw::to_string(i),
      "cum_pose_orientation_exploring_robot_" + rcppsw::to_string(i),
      "cum_pose_position_exploring_robot_" + rcppsw::to_string(i),

      "cum_traveled_flocking_robot_" + rcppsw::to_string(i),
      "cum_twist_linear_velocity_flocking_robot_" + rcppsw::to_string(i),
      "cum_twist_angular_velocity_flocking_robot_" + rcppsw::to_string(i),
      "cum_pose_orientation_flocking_robot_" + rcppsw::to_string(i),
      "cum_pose_position_flocking_robot_" + rcppsw::to_string(i),

      "cum_traveled_all_robot_" + rcppsw::to_string(i),
      "cum_twist_linear_velocity_all_robot_" + rcppsw::to_string(i),
      "cum_twist_angular_velocity_all_robot_" + rcppsw::to_string(i),
      "cum_pose_orientation_all_robot_" + rcppsw::to_string(i),
      "cum_pose_position_all_robot_" + rcppsw::to_string(i),
      /* clang-format on */
    };
    merged.splice(merged.end(), cols);
  }
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
kinematics_metrics_dist_csv_sink::csv_line_build(const rmetrics::base_data* data,
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

std::string kinematics_metrics_dist_csv_sink::build_from_context(
    const context_type& ctx,
    const kinematics_metrics_data* data,
    const rtypes::timestep& t) {
  std::string line;

  for (size_t i = 0; i < data->n_robots(); ++i) {
    line += csv_entry_domavg(data->interval()[i].traveled[ctx].v(),
                             output_interval().v());
    line += csv_entry_domavg(data->interval()[i].twist[ctx].linear,
                             output_interval().v());
    line += csv_entry_domavg(data->interval()[i].twist[ctx].angular,
                             output_interval().v());
    line += csv_entry_domavg(data->interval()[i].pose[ctx].orientation,
                             output_interval().v());
    line += csv_entry_domavg(data->interval()[i].pose[ctx].position,
                             output_interval().v());

    line += csv_entry_domavg(data->cum()[i].traveled[ctx].v(), t.v());
    line += csv_entry_domavg(data->cum()[i].twist[ctx].linear, t.v());
    line += csv_entry_domavg(data->cum()[i].twist[ctx].angular, t.v());
    line += csv_entry_domavg(data->cum()[i].pose[ctx].orientation, t.v());
    line += csv_entry_domavg(data->cum()[i].pose[ctx].position, t.v());

  } /* for(i..) */

    return line;
} /* build_from_context() */

} /* namespace cosm::kin::metrics */
