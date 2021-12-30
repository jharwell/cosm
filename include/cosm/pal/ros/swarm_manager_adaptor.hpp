/**
 * \file swarm_manager_adaptor.hpp
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

#ifndef INCLUDE_COSM_PAL_ROS_SWARM_MANAGER_ADAPTOR_HPP_
#define INCLUDE_COSM_PAL_ROS_SWARM_MANAGER_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/time.h>

#include "cosm/pal/base_swarm_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, ros);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class swarm_manager_adaptor
 * \ingroup pal ros
 *
 * \brief Adaptor for \ref base_swarm_manager to provide an interface for
 * managing swarms within ROS.
 */
class swarm_manager_adaptor : public cpal::base_swarm_manager,
                              public rer::client<swarm_manager_adaptor> {
 public:
  swarm_manager_adaptor(void);
  ~swarm_manager_adaptor(void) override;

  /* Not copy constructable/assignable by default */
  swarm_manager_adaptor(const swarm_manager_adaptor&) = delete;
  const swarm_manager_adaptor& operator=(const swarm_manager_adaptor&) = delete;

  /* swarm_manager overrides */
  void init(ticpp::Element&) override {}
  void pre_step(void) override {}
  void reset(void) override {}
  void post_step(void) override {}
  void destroy(void) override {}

 protected:
#if (LIBRA_ER >= LIBRA_ER_ALL)
  void ndc_push(void) const override final {
    auto diff = ::ros::Time::now().sec - m_start.sec;
    ER_NDC_PUSH("[t=" + rcppsw::to_string(diff) +"." "]");
  }
  void ndc_pop(void) const override final { ER_NDC_POP(); }
#else
  void ndc_push(void) const override final {}
  void ndc_pop(void) const override final {}
#endif

 private:
  /* clang-format off */
  ::ros::Time m_start{::ros::Time::now()};
  /* clang-format on */
};

NS_END(ros, pal, cosm);

#endif /* INCLUDE_COSM_PAL_ROS_SWARM_MANAGER_ADAPTOR_HPP_ */
