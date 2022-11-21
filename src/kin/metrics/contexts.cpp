/**
 * \file contexts.cpp
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/kin/metrics/contexts.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::kin::metrics {

/*******************************************************************************
 * Kernel Functions
 ******************************************************************************/
std::vector<rmetrics::context> kContexts = {
  rmetrics::context(context_type::ekHOMING),
  rmetrics::context(context_type::ekEXPLORING),
  rmetrics::context(context_type::ekFLOCKING),
  rmetrics::context(context_type::ekALL),
};

} /* namespace cosm::kin::metrics */
