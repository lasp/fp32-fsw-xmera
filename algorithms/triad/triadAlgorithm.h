#ifndef F32XMERA_TRIAD_ALGORITHM_H
#define F32XMERA_TRIAD_ALGORITHM_H

#include "triadTypes.h"

#include <Eigen/Core>

class TriadAlgorithm final {
   public:
    Eigen::Vector3f update(const Eigen::Vector3f& rHat_SB_N,
                           const Eigen::Vector3f& hReqHat_N,
                           const Eigen::Vector3f& hRefHat_B) const;

    void setA1Hat_B(const Eigen::Vector3f& a1Hat_B);
    Eigen::Vector3f getA1Hat_B() const;

   private:
    Eigen::Vector3f a1Hat_B = Eigen::Vector3f::Zero();
};

#endif
