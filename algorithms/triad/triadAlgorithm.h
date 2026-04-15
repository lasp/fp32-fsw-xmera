#ifndef F32XMERA_TRIAD_ALGORITHM_H
#define F32XMERA_TRIAD_ALGORITHM_H

#include "triadTypes.h"

#include <Eigen/Core>

class TriadAlgorithm final {
   public:
    Eigen::Vector3d update(const Eigen::Vector3d& rHat_SB_N,
                           const Eigen::Vector3d& hReqHat_N,
                           const Eigen::Vector3d& hRefHat_B) const;

    void setA1Hat_B(const Eigen::Vector3d& a1Hat_B);
    Eigen::Vector3d getA1Hat_B() const;

   private:
    Eigen::Vector3d a1Hat_B = Eigen::Vector3d::Zero();
};

#endif
