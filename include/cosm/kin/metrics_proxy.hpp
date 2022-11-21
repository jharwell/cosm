/**
 * \file metrics_proxy.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/kin/metrics/kinematics_metrics.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::kin {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class metrics_proxy
 * \ingroup kin
 *
 * \brief Proxy to handle agent kinematics for the purpose of metric collection.
 *
 * This class makes the resulting controller classes simpler in that they don't
 * have a bunch of functions exposed which have no purposes other than to use to
 * collect metrics.
 */
class metrics_proxy final : public ckmetrics::kinematics_metrics {
 public:
  using context_cb = std::function<bool(const rmetrics::context&)>;
  metrics_proxy(const rtypes::type_uuid& agent_id,
                const csubsystem::sensing_subsystem* const sensing,
                context_cb cb)
      : mc_id(agent_id),
        mc_sensing(sensing),
        mc_ctx_cb(cb) {}

  /* Not copy/move constructable/assignable by default */
  metrics_proxy(const metrics_proxy&) = delete;
  metrics_proxy& operator=(const metrics_proxy&) = delete;
  metrics_proxy(metrics_proxy&&) = delete;
  metrics_proxy& operator=(metrics_proxy&&) = delete;

  /* movement metrics */
  boost::optional<rspatial::euclidean_dist>
  traveled(const rmetrics::context& ctx) const override final;

  boost::optional<ckin::twist>
  twist(const rmetrics::context& ctx) const override final;

  ckin::pose pose(void) const override final;
  const rtypes::type_uuid& id(void) const override final { return mc_id; }

  /* clang-format off */
  const rtypes::type_uuid                 mc_id;
  const csubsystem::sensing_subsystem*    mc_sensing;
  const context_cb                        mc_ctx_cb;
  /* clang-format off */
};

} /* namespace cosm::kin */
