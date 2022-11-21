/**
 * \file kinematics_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/kin/metrics/kinematics_metrics_collector.hpp"

#include <functional>

#include "cosm/kin/metrics/kinematics_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::kin::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
kinematics_metrics_collector::kinematics_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink,
    size_t n_robots,
    size_t n_contexts)
    : ER_CLIENT_INIT("cosm.kin.metrics.kinematics_metrics_collector"),
      base_collector(std::move(sink)),
      m_data(n_robots, n_contexts) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void kinematics_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const kinematics_metrics&>(metrics);

  for (size_t i = 0; i < m_data.n_contexts(); ++i) {
    auto ctx = rmetrics::context(i);
    if (auto traveled = m.traveled(ctx)) {
      m_data.interval()[m.id().v()].traveled[ctx.v()] += *traveled;
    }
    if (auto twist = m.twist(ctx)) {
      m_data.interval()[m.id().v()].twist[ctx.v()] += *twist;
    }

    m_data.interval()[m.id().v()].pose[ctx.v()] += m.pose();

    if (auto traveled = m.traveled(ctx)) {
      m_data.cum()[m.id().v()].traveled[ctx.v()] += *traveled;
    }
    if (auto twist = m.twist(ctx)) {
      m_data.cum()[m.id().v()].twist[ctx.v()] += *twist;
    }

    m_data.cum()[m.id().v()].pose[ctx.v()] += m.pose();
  } /* for(i..) */
} /* collect() */

void kinematics_metrics_collector::reset_after_interval(void) {
  for (size_t i = 0; i < m_data.n_robots(); ++i) {
    for (size_t j = 0; j < m_data.n_contexts(); ++j) {
      m_data.interval()[i].traveled[j] = rspatial::euclidean_dist(0);
      m_data.interval()[i].twist[j] = {};
      m_data.interval()[i].pose[j] = {};
    } /* for(j..) */
  } /* for(i..) */
} /* reset_after_interval() */

} /* namespace cosm::kin::metrics */
