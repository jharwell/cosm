/**
 * \file los2d.hpp
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

#ifndef INCLUDE_COSM_REPR_LOS2D_HPP_
#define INCLUDE_COSM_REPR_LOS2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/base_los.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class los2D
 * \ingroup repr
 *
 * \brief A repr of the robot's current line-of-sight in 2D.
 */
class los2D : public crepr::base_los<cds::cell2D>,
              public rer::client<los2D> {
 public:
  explicit los2D(const const_grid_view& c_view)
      : base_los(c_view),
        ER_CLIENT_INIT("cosm.repr.los2D") {}

  const cds::cell2D& access(size_t i, size_t j) const override;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_LOS2D_HPP_ */
