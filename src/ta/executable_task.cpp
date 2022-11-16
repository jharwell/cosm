/**
 * \file executable_task.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/executable_task.hpp"

#include "rcppsw/math/config/ema_config.hpp"

#include "cosm/ta/config/src_sigmoid_sel_config.hpp"
#include "cosm/ta/time_estimate.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
executable_task::executable_task(const std::string& name,
                                 const config::src_sigmoid_sel_config* abort,
                                 const rmath::config::ema_config* estimation)
    : logical_task(name),
      ER_CLIENT_INIT("cosm.ta.executable_task"),
      mc_abort_src(abort->input_src),
      m_interface_in_prog(kMAX_INTERFACES, false),
      m_interface_times(kMAX_INTERFACES, rtypes::timestep(0)),
      m_last_interface_times(kMAX_INTERFACES, rtypes::timestep(0)),
      m_interface_start_times(kMAX_INTERFACES, rtypes::timestep(0)),
      m_interface_estimates(kMAX_INTERFACES, time_estimate(estimation->alpha)),
      m_exec_estimate(estimation->alpha),
      m_abort_prob(&abort->sigmoid.sigmoid) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
int executable_task::active_interface(void) const {
  for (size_t i = 0; i < m_interface_in_prog.size(); ++i) {
    if (m_interface_in_prog[i]) {
      return static_cast<int>(i);
    }
  } /* for(i..) */

  return -1;
} /* active_interface() */

NS_END(ta, cosm);
