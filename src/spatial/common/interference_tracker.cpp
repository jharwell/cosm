/**
 * \file interference_tracker.cpp
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/common/interference_tracker.hpp"


/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<rtypes::timestep> interference_tracker::interference_duration(void) const {
  if (exited_interference()) {
    return boost::make_optional(state_tracker::state_duration());
  }
  return boost::none;
}
boost::optional<rmath::vector3z> interference_tracker::interference_loc3D(void) const {
  if (exp_interference()) {
    return boost::make_optional(state_tracker::state_loc3D());
  }
  return boost::none;
}

} /* namespace cosm::spatial */
