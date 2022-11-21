/**
 * \file output_config.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/config/output_config.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal::config {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
fs::path output_config::root_calc(const output_config* const config) {
  fs::path path = config->output_parent;

  if ("__current_date__" == config->output_leaf) {
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    path /= rcppsw::to_string(now.date().year()) + "-" +
            rcppsw::to_string(now.date().month()) + "-" +
            rcppsw::to_string(now.date().day()) + ":" +
            rcppsw::to_string(now.time_of_day().hours()) + "-" +
            rcppsw::to_string(now.time_of_day().minutes());
  } else {
    path /= config->output_leaf;
  }
  return path;
} /* root_calc */

} /* namespace cosm::pal::config */
