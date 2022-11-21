.. SPDX-License-Identifier: MIT

.. _ln-cosm-usage:

=================================
Using COSM: Runtime Configuration
=================================

All of COSM's runtime configuration parameters are read from XML (for the
moment). Its structure supports other types of sources.

XML Configuration
=================

Configuration is divided into two parts: configuration for the controllers which
run on each agent, and configuration for the swarm manager, which is what runs
"above" all agents (either real or simulated) and does tasks like collecting
data from agents and modifying the simulation environment when certain events
occur.

XML Conventions
---------------

COSM Uses the following conventions when describing its XML configuration:

- Multiple choices for an XML attribute value are separated by a ``|`` in the
  example XML.

- XML attributes that should be floating point are specified as ``FLOAT`` in the
  example XML (acceptable range, if applicable, is documented for each
  individual attribute).

- XML attributes that should be integers are specified as ``INTEGER`` in the
  example XML (acceptable range, if applicable, is documented for each
  individual attribute).

.. toctree::
   :maxdepth: 2
   :caption: XML Configuration Details:

   controllers.rst
   swarm-manager.rst
