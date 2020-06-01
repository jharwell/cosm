/**
 * \file base_expstrat.cpp
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
#include "cosm/spatial/expstrat/base_expstrat.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, expstrat);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_expstrat::base_expstrat(params* const p)
    : base_expstrat{p->saa, p->rng} {}

base_expstrat::base_expstrat(subsystem::saa_subsystemQ3D* const saa,
                             rmath::rng* rng)
    : m_saa(saa), m_rng(rng), m_inta_tracker(m_saa->sensing())  {}

NS_END(expstrat, spatial, cosm);
