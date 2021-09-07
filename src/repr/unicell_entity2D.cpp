/**
 * \file unicell_entity2D.cpp
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
#include "cosm/repr/unicell_entity2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
unicell_entity2D::unicell_entity2D(const rtypes::type_uuid& id,
                                   const rmath::vector2d& rdim,
                                   const rtypes::discretize_ratio& resolution,
                                   const rmath::vector2d& rcenter)
    : entity2D(id),
      ER_CLIENT_INIT("cosm.repr.unicell_entity2D"),
      mc_arena_res(resolution),
      m_rdim(rdim),
      m_rcenter(rcenter),
      m_ranchor(m_rcenter - m_rdim / 2.0),
      m_ddim(rmath::dvec2zvec(m_rdim, mc_arena_res.v())),
      m_dcenter(rmath::dvec2zvec(m_rcenter, mc_arena_res.v())),
      m_danchor(m_dcenter - m_ddim / 2) {}

NS_END(repr, cosm);
