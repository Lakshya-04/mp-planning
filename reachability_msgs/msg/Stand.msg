# One base stand: where the arm base and the base stand, and what is reached from there.
float64 x                       # arm base position (world)
float64 y
float64 yaw                     # base yaw (0 without a base model)
float64 standoff                # arm base distance from the wall plane
float64 footprint_clearance     # footprint to wall distance (0 without a base model)
float64 travel_s                # driving time from the previous stand (0 without a base model)
uint32[] target_indices         # indices into the request's targets_world reached from here
reachability_msgs/LiftBand[] bands   # lift bands for those targets (pose_indices are indices into the request's targets_world)
