/**
 * \file robot_manager_adaptor.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/time.h>

#include "cosm/pal/base_swarm_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ros {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_manager_adaptor
 * \ingroup ros
 *
 * \brief Adaptor for \ref cpal::base_swarm_manager to provide an interface for
 * managing a single robot within a swarm using ROS.
 */
class robot_manager_adaptor : public cpal::base_swarm_manager,
                              public rer::client<robot_manager_adaptor> {
 public:
  robot_manager_adaptor(void)
      : ER_CLIENT_INIT("cosm.ros.robot_manager_adaptor") {}
  ~robot_manager_adaptor(void) override = default;

  /* Not copy constructable/assignable by default */
  robot_manager_adaptor(const robot_manager_adaptor&) = delete;
  const robot_manager_adaptor& operator=(const robot_manager_adaptor&) = delete;

  /**
   * \brief Is it time for the experiment to end?
   */
  virtual bool experiment_finished(void) const = 0;

  /* robot_manager overrides */
  void init(ticpp::Element&) override {}
  void pre_step(void) override {}
  void reset(void) override {}
  void post_step(void) override {}
  void destroy(void) override {}

 protected:
#if (RCPPSW_ER >= RCPPSW_ER_ALL)
  void ndc_uuid_push(void) const override final { ER_NDC_PUSH("[ros_rb]"); }
  void ndc_uuid_pop(void) const override final { ER_NDC_POP(); }
  void mdc_ts_update(void) const override final {
    auto diff = ::ros::Time::now().sec - m_start.sec;
    ER_MDC_RM("time");
    ER_MDC_ADD("time", "[t=" + rcppsw::to_string(diff) + "]");
  }
#else
  void ndc_uuid_push(void) const override final {}
  void ndc_uuid_pop(void) const override final {}
  void mdc_ts_update(void) const override final {}
#endif

 private:
  /* clang-format off */
  ::ros::Time m_start{::ros::Time::now()};
  /* clang-format on */
};

} /* namespace cosm::ros */
