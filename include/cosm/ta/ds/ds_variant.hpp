/**
 * \file ds_variant.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_TA_DS_DS_VARIANT_HPP_
#define INCLUDE_COSM_TA_DS_DS_VARIANT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>

#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, ds);
class bi_tdgraph;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief boost::variant containing one of the possible datastructures that the
 * \ref base_executive and its derived classes can operate on.
 */
using ds_variant = boost::variant<bi_tdgraph>;

NS_END(ds, ta, cosm);

#endif /* INCLUDE_COSM_TA_DS_DS_VARIANT_HPP_ */
