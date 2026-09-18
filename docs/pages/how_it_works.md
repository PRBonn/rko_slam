# How it works

rko_slam consumes the scan stream and the odometry TF of a running odometry (rko_lio by default, or anything
locally consistent publishing `odom <- base`) and publishes a `map <- odom` correction. The pipeline is
essentially a reimplementation of [KISS-SLAM](https://github.com/PRBonn/kiss-slam), uses
[MapClosures](https://github.com/PRBonn/MapClosures) for detecting revisits, and a g2o pose graph.

## Sub-maps

The system does not maintain a single global map but splits the trajectory into local maps, or sub-maps. Each
sub-map is anchored at a keypose, the pose estimate at which it is opened, and holds a voxel grid of the local
map points expressed in the keypose frame. Each new deskewed scan is integrated into the open sub-map using its
odometry pose, looked up on TF at the scan's timestamp. Points closer than `min_range` or further than
`max_range` never enter, `voxel_size` sets the cell, and `max_points_per_voxel` caps how many points a cell
keeps.

Each sub-map is bounded by a splitting criterion: the displacement of the current pose from the sub-map's
keypose. Once that crosses `splitting_distance`, the current sub-map is closed and a new one opened, its keypose
seeded from the composition of the old keypose and the odometry accumulated since, breaking the trajectory into
locally consistent segments. This is straight-line displacement, not distance travelled - a 700 m loop that
never gets more than 80 m from where it started is one sub-map, not seven.

## The pose graph

The sub-map keyposes form the nodes of the pose graph, connected by the relative odometry between consecutive
keyposes as edge constraints, so that correcting the keyposes corrects the whole trajectory. That chain is the
odometry, just sampled at keyposes instead of at scans.

The odometry does not provide an uncertainty estimate for these poses. As in KISS-SLAM and other LiDAR SLAM
systems, a fixed covariance is used for the odometry edges, since reliable covariance estimates for scan
matching are hard to obtain in practice. Each edge is weighted by a block-diagonal information matrix with
isotropic translation and rotation blocks, the rotation block set to a fixed multiple of the translation block.
That multiple is `rotation_info_scale`, and it accounts for the scale difference between the metre-scale
translation and radian-scale rotation residuals.

## Finding a revisit

Whenever a sub-map is completed, the system searches for loop closures between it and all previously built
sub-maps. That search runs on its own thread, so the scan stream keeps moving while it works.

MapClosures identifies the ground points in the local map, projects the map onto the ground plane into a bird's
eye view density image (`density_map_resolution` is its cell size, `density_threshold` decides when a cell
counts as occupied), and computes binary ORB descriptors that are matched against the descriptors of all
previous sub-maps. `hamming_distance_threshold` is how different two descriptors may be and still be called a
match, and `no_of_sub_maps_to_skip` keeps the most recent sub-maps out of the search so a sub-map does not match
its own neighbours.

A matching candidate is geometrically validated with RANSAC to obtain an initial alignment, and
`inliers_threshold` is how much agreement that alignment needs before it is worth checking properly. Checking it
properly means refining the alignment with point-to-plane ICP between the two sub-maps' voxel centroids, into a
relative closure transform that maps the keypose frame of one sub-map into that of the other.

The overlap between the two sub-maps is then measured with the overlap coefficient: the number of voxels the
two share after aligning, divided by the size of the smaller sub-map. This ratio lies in 0 to 1, and
is close to zero when the sub-maps barely overlap and one when the smaller sub-map lies entirely within the
other. That is what `overlap_threshold` gates.

## Closing the loop

An accepted closure is inserted as a new edge, with a Cauchy robust kernel on it to down-weight inconsistent or
outlier closures, and the graph is re-solved (Dogleg, Cholmod) immediately. The keyposes move, the trajectory is
rebuilt from them, and the difference between where the odometry thinks you are and where the graph now says you
are is published as `map <- odom`.

## Across sessions

<div class="pair">
  <figure><figcaption>as recorded</figcaption><img class="only-dark" src="../_static/img/multi_session_recorded_dark.png" alt="three sessions, each in its own frame"><img class="only-light" src="../_static/img/multi_session_recorded_light.png" alt="three sessions, each in its own frame"></figure>
  <figure><figcaption>aligned</figcaption><img class="only-dark" src="../_static/img/multi_session_aligned_dark.png" alt="the same three sessions in one frame"><img class="only-light" src="../_static/img/multi_session_aligned_light.png" alt="the same three sessions in one frame"></figure>
</div>
<p class="pair-caption">Three sessions of the same place, recorded on different days, each starting in its own frame, and after alignment.</p>

Every run ends in its own frame, wherever the odometry happened to start. A robot often operates in the same
place several times on different days, or a single mission is interrupted and resumed later. Multi-session
alignment treats each continuous operation as one session and merges several of them into one common frame,
reusing the single-session components and detecting inter-session closures through the same place recognition.

It runs offline on the artifacts each completed session produces, the sub-maps with their keyposes and the
per-session pose graph, without reprocessing the raw sensor data. The closure detector is rebuilt from the
stored sub-maps and searched for closures between sessions, since the closures within each session are already
part of its own pose graph. The inter-session closures are validated with the same overlap criterion as before.

Each session is internally consistent in its own frame, and a single inter-session closure fixes that session's
frame relative to a neighbour's. The sessions and their accepted inter-session closures form a graph, over which
a spanning tree is computed, rooted at the session that reaches the most others. Composing the closure
transforms along each tree path places every session in the common frame, giving an initial guess for its
keyposes. Any session with no closure path to the root is left out, with a warning, since without such a link
its trajectory cannot be brought into the common frame.

A single pose graph is then built over the keyposes of all sessions, adding each session's own odometry and
closures together with every accepted inter-session closure as edges. The first keypose of the root session is
fixed, removing the gauge freedom. Optimizing this joint graph yields the merged set of trajectories in the
common frame, which is what the run directory holds. Beyond placing the sessions in one frame, the joint
optimization can also improve the trajectory consistency within each session.

The closure parameters are the same as for a single run, except `no_of_sub_maps_to_skip`, which is 0 here since
neighbouring sub-maps from different sessions are real candidates. `overlap_threshold` is the one that matters:
where the physical overlap between two recordings is small, lower it accordingly. How to run it is on the
{doc}`Build and run <build_and_run>` page.
