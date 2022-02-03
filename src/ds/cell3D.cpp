/**
 * \file cell3D.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/cell3D.hpp"

#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell3D::cell3D(void) { decoratee().init(); }

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
crepr::sim_block3D* cell3D::block(void) const {
  return dynamic_cast<crepr::sim_block3D*>(m_entity);
} /* block3D() */

crepr::sim_block3D* cell3D::block(void) {
  return dynamic_cast<crepr::sim_block3D*>(m_entity);
} /* block3D() */

NS_END(ds, cosm);
