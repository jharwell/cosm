/**
 * \file kinematics_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/metrics/base_data.hpp"
#include "rcppsw/al/multithread.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/kin/pose.hpp"
#include "cosm/kin/twist.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::kin::metrics {

namespace detail {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct robot_kinematics_context_data
 * \ingroup kin metrics detail
 *
 * \brief Container for holding collected statistics of \ref
 *        kinematics_metrics for a single robot.
 *
 * Does not have to be atomic because the vectors are indexed by robot ID, which
 * is unique, and exactly 1 thread will update a given robot's entry in a given
 * vector each timestep.
 */
struct robot_kinematics_context_data {
  explicit robot_kinematics_context_data(size_t n_contexts)
      : traveled(n_contexts, rspatial::euclidean_dist(0.0)),
        twist(n_contexts),
        pose(n_contexts) {}

    /* clang-format off */
  std::vector<rspatial::euclidean_dist> traveled;
  std::vector<ckin::twist>              twist;
  std::vector<ckin::pose>               pose;
  /* clang-format on */
};

} /* namespace detail */

class kinematics_metrics_data : public rmetrics::base_data {
 public:
  explicit kinematics_metrics_data(size_t n_robots,
                                   size_t n_contexts)
      : m_n_robots(n_robots),
        m_n_contexts(n_contexts),
        m_interval(n_robots, detail::robot_kinematics_context_data(n_contexts)),
        m_cum(n_robots, detail::robot_kinematics_context_data(n_contexts)) {}

  size_t n_robots(void) const { return m_n_robots; }
  size_t n_contexts(void) const { return m_n_contexts; }
  auto& interval(void) { return m_interval; }
  auto& cum(void) { return m_cum; }
  const auto& interval(void) const { return m_interval; }
  const auto& cum(void) const { return m_cum; }

  /**
   * \brief Accumulate data. We ignore the "cum" field on \p rhs, and accumulate
   * into our "cum" field using the "interval" field of \p rhs.
   *
   * This is the most meaningful semantics I could come up with; I couldn't find
   * a way to justify accumulating already cumulative data again (it would have
   * required some additional changes/contortions elsewhere).
   */
  kinematics_metrics_data& operator+=(const kinematics_metrics_data& rhs) {
    /* for each robot's data */
    for (size_t i = 0; i < n_robots(); ++i) {
      /* for each category of kinematics */
      for (size_t j = 0; j < n_contexts(); ++j) {
        m_interval[i].traveled[j] += rhs.m_interval[i].traveled[j];
        m_interval[i].twist[j] += rhs.m_interval[i].twist[j];
        m_interval[i].pose[j] += rhs.m_interval[i].pose[j];
      } /* for(j..) */
    } /* for(i..) */
    return *this;
  }

 private:
  /* clang-format off */
  size_t                                             m_n_robots;
  size_t                                             m_n_contexts;
  std::vector<detail::robot_kinematics_context_data> m_interval;
  std::vector<detail::robot_kinematics_context_data> m_cum;
  /* clang-format on */
};

} /* namespace cosm::kin::metrics */
