#ifndef XMERA_EIGEN_FUZZ_DOMAINS_HPP
#define XMERA_EIGEN_FUZZ_DOMAINS_HPP

#include <fuzztest/fuzztest.h>
#include <Eigen/Core>
#include <utility>
#include <vector>

namespace xmera::fuzz {

template <typename Scalar, int Size, typename InnerDomain>
inline auto EigenVectorOf(InnerDomain inner) {
    using VectorT = Eigen::Matrix<Scalar, Size, 1>;
    return fuzztest::Map([](const std::vector<Scalar>& v) -> VectorT { return Eigen::Map<const VectorT>(v.data()); },
                         fuzztest::VectorOf(std::move(inner)).WithSize(Size));
}

inline auto Vector3dInRange(double min, double max) { return EigenVectorOf<double, 3>(fuzztest::InRange(min, max)); }

inline auto Vector3fInRange(float min, float max) { return EigenVectorOf<float, 3>(fuzztest::InRange(min, max)); }

inline auto Vector3dFinite() { return EigenVectorOf<double, 3>(fuzztest::Finite<double>()); }

inline auto Vector3fFinite() { return EigenVectorOf<float, 3>(fuzztest::Finite<float>()); }

}  // namespace xmera::fuzz

#endif  // XMERA_EIGEN_FUZZ_DOMAINS_HPP
