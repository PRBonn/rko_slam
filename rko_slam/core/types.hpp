#pragma once

#include <cstdint>

#include <rko_lio/core/util.hpp>
#include <sophus/se3.hpp>

namespace rko_slam::core {

using rko_lio::core::Nsec;

using KeyposeId = std::uint64_t;

struct TrajectorySample {
  Nsec time{0};
  Sophus::SE3f pose;
};

} // namespace rko_slam::core
