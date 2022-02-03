/**
 * \file block3D_manifest_processor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_FORAGING_BLOCK_DIST_BLOCK3D_MANIFEST_PROCESSOR_HPP_
#define INCLUDE_COSM_FORAGING_BLOCK_DIST_BLOCK3D_MANIFEST_PROCESSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>
#include <memory>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/factory/factory.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/foraging/config/block_manifest.hpp"
#include "cosm/ds/block3D_vector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block3D_manifest_processor
 * \ingroup foraging block_dist
 *
 * \brief Translates the parsed XML configuration for how many/what type of
 * blocks should be used in simulation into a heterogeneous vector of actual
 * blocks.
 */
class block3D_manifest_processor
    : public rpfactory::releasing_factory<crepr::sim_block3D,
                                        std::string, /* key type */
                                        const rtypes::type_uuid&,
                                        const rmath::vector3d&,
                                        const rtypes::discretize_ratio&> {
 public:
  explicit block3D_manifest_processor(const config::block_manifest* m,
                                      const rtypes::discretize_ratio& arena_res);

  cds::block3D_vectoro operator()(void);

 private:
  /* clang-format off */
  const rtypes::discretize_ratio mc_arena_res;
  const config::block_manifest   mc_manifest;
  /* clang-format on */
};

NS_END(block_dist, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_BLOCK_DIST_BLOCK3D_MANIFEST_PROCESSOR_HPP_ */
