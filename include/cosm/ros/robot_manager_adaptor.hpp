/**
 * \file robot_manager_adaptor.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_ROS_ROBOT_MANAGER_ADAPTOR_HPP_
#define INCLUDE_COSM_ROS_ROBOT_MANAGER_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/time.h>

#include "cosm/pal/base_swarm_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_manager_adaptor
 * \ingroup ros
 *
 * \brief Adaptor for \ref base_swarm_manager to provide an interface for
 * managing a single robot within a swarm using ROS.
 */
class robot_manager_adaptor : public cpal::base_swarm_manager,
                              public rer::client<robot_manager_adaptor> {
 public:
  robot_manager_adaptor(void);
  ~robot_manager_adaptor(void) override;

  /* Not copy constructable/assignable by default */
  robot_manager_adaptor(const robot_manager_adaptor&) = delete;
  const robot_manager_adaptor& operator=(const robot_manager_adaptor&) = delete;

  /* robot_manager overrides */
  void init(ticpp::Element&) override;
  void pre_step(void) override {}
  void reset(void) override {}
  void post_step(void) override {}
  void destroy(void) override {}

 protected:
#if (LIBRA_ER >= LIBRA_ER_ALL)
  void ndc_uuid_push(void) const override final {
    ER_NDC_PUSH("[ros_rb]");
  }
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

NS_END(ros, cosm);

#endif /* INCLUDE_COSM_ROS_ROBOT_MANAGER_ADAPTOR_HPP_ */
