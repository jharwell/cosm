/**
 * \file topic_sink.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_COSM_ROS_METRICS_TOPIC_SINK_HPP_
#define INCLUDE_COSM_ROS_METRICS_TOPIC_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include <ros/ros.h>

#include "rcppsw/metrics/network_sink.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class topic_sink
 * \ingroup ros metrics
 *
 * \brief Metrics sink so that collectors can output their metrics to a ROS
 * topic. Metrics are written every timestep so they are available for the
 * central swarm manager node to collect and process.
 */
template<typename TMsgData>
class topic_sink : public rmetrics::network_sink,
                   public rer::client<topic_sink<TMsgData>> {
 public:
  topic_sink(const std::string& topic,
             const rmetrics::output_mode& mode,
             const rtypes::timestep& interval)
      : network_sink(topic, mode, interval),
        ER_CLIENT_INIT("cosm.pal.ros.metrics.topic_sink") {}

  virtual ~topic_sink(void) = default;

  /* network_sink overrides */
  void initialize(const rmetrics::base_data*) override {
    m_pub = ::ros::NodeHandle().advertise<TMsgData>(topic(),
                                                    kQueueBufferSize);
  }


  rmetrics::write_status flush(const rmetrics::base_data* data,
                               const rtypes::timestep&) override {
    m_pub.publish(*(static_cast<const TMsgData*>(data)));
    return rmetrics::write_status::ekSUCCESS;
  } /* flush() */

  void finalize(void) override final {
    m_pub.~Publisher();
  }

 protected:
  /**
   * \brief If messages not consumed quickly enough, buffer up this many
   * messages before throwing some away.
   */
  static constexpr const size_t kQueueBufferSize = 1000;

  const ::ros::Publisher* pub(void) const { return &m_pub; }
  void pub(const ::ros::Publisher& pub) { m_pub = pub; }
  const std::string& topic(void) const { return network_sink::dest(); }

 private:
  /* clang-format off */
  ::ros::Publisher  m_pub{};
  /* clang-format on */
};

NS_END(metrics, ros, cosm);

#endif /* INCLUDE_COSM_ROS_METRICS_TOPIC_SINK_HPP_ */