/**
 * \file losQ3D.hpp
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

#ifndef INCLUDE_COSM_REPR_LOSQ3D_HPP_
#define INCLUDE_COSM_REPR_LOSQ3D_HPP_

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
 * \class losQ3D
 * \ingroup repr
 *
 * \brief A repr of the robot's current line-of-sight in quasi-3D. "Quasi"
 * because the LOS object itself is only 2D, BUT contains information about a
 * slice of 3D cells. This is in keeping with making the robot controllers as
 * simple as possible.
 */
class losQ3D : public crepr::base_los<cds::cell3D>,
               public rer::client<losQ3D> {
 public:
  explicit losQ3D(const const_grid_view& c_view);
  const cds::cell3D& access(size_t i, size_t j) const override;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_LOSQ3D_HPP_ */
