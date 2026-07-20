#include "inertial3DAlgorithm.h"

Inertial3DAlgorithm::Inertial3DAlgorithm(const Inertial3DConfig& config) : cfg(config) { setConfig(config); }

void Inertial3DAlgorithm::setConfig(const Inertial3DConfig& config) { this->cfg = config; }

/*! This method returns the fixed reference attitude. The desired orientation is defined by the configuration.
 @return Eigen::Vector3f  MRP from inertial frame N to reference frame R
 */
Eigen::Vector3f Inertial3DAlgorithm::update() const { return this->cfg.getSigmaRN(); }
