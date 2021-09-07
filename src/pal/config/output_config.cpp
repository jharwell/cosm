/**
 * \file output_config.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/config/output_config.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, config);

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

NS_END(config, pal, cosm);
