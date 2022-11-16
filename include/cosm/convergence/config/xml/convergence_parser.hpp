/**
 * \file convergence_parser.hpp
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
#include <memory>

#include "cosm/convergence/config/convergence_config.hpp"
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/convergence/config/xml/positional_entropy_parser.hpp"
#include "cosm/convergence/config/xml/task_dist_entropy_parser.hpp"
#include "cosm/convergence/config/xml/interactivity_parser.hpp"
#include "cosm/convergence/config/xml/angular_order_parser.hpp"
#include "cosm/convergence/config/xml/velocity_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class convergence_parser
 * \ingroup convergence config xml
 *
 * \brief Parses XML configuration related to calculating swarm convergence into
 * \ref convergence_config.
 */
class convergence_parser final : public rer::client<convergence_parser>,
                                 public rconfig::xml::xml_config_parser {
 public:
  using config_type = convergence_config;

  convergence_parser(void) : ER_CLIENT_INIT("cosm.convergence.config.xml.convergence_parser") {}

  ~convergence_parser(void) override = default;

  /**
   * \brief The root tag that all XML configuration relating to convergence
   * parameters should lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "convergence";

  bool validate(void) const override RCPPSW_ATTR(const, cold);
  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  positional_entropy_parser    m_pos_entropy{};
  task_dist_entropy_parser     m_task_entropy{};
  interactivity_parser         m_interactivity{};
  angular_order_parser         m_ang_order{};
  velocity_parser              m_velocity{};
  /* clang-format on */
};

NS_END(xml, config, convergence, cosm);

