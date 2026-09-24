#pragma once

#include <filesystem>

#include "rko_slam/pgo/pose_graph.hpp"

namespace rko_slam::pgo {

// g2o text, with EDGE_GRAVITY lines and a closure comment of our own.
bool save(const PoseGraph& pose_graph, const std::filesystem::path& path);

// Throws rko_lio::core::InputError on a file it cannot open or parse.
// The graph comes back with a default config; the run's config.yaml has the one it was solved under.
PoseGraph load(const std::filesystem::path& path);

} // namespace rko_slam::pgo
