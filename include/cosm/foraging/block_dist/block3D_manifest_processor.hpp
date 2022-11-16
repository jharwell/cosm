/**
 * \file block3D_manifest_processor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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

