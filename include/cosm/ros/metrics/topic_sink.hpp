/**
 * \file topic_sink.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include <ros/ros.h>

#include "rcppsw/metrics/network_sink.hpp"

#include "cosm/ros/metrics/msg_traits.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ros::metrics {

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
template<typename TMsg>
class topic_sink : public rmetrics::network_sink,
                   public rer::client<topic_sink<TMsg>> {
 public:
  topic_sink(const std::string& topic,
             const rmetrics::output_mode& mode,
             const rtypes::timestep& interval)
      : network_sink(topic, mode, interval),
        ER_CLIENT_INIT("cosm.ros.metrics.topic_sink") {}

  virtual ~topic_sink(void) = default;

  /* network_sink overrides */
  void initialize(const rmetrics::base_data*) override {
    m_pub = ::ros::NodeHandle().advertise<TMsg>(topic(),
                                                kQueueBufferSize);
  }


  rmetrics::write_status flush(const rmetrics::base_data* data,
                               const rtypes::timestep&) override {
    static_assert(rmpl::is_complete_type<
                  msg_traits::payload_type<TMsg>>::value,
                  "payload_type<> must be specialized for TMsg");

    using payload_type = typename msg_traits::payload_type<TMsg>::type;
    TMsg msg;
    msg.data = *(static_cast<const payload_type*>(data));
    m_pub.publish(msg);
    ER_DEBUG("Published '%s' metrics data", topic().c_str());
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

} /* namespace cosm::ros::metrics */
