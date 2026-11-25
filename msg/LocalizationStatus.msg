# What the localizer knows about its own pose estimate, as facts and nothing
# else. Published by localizer_node (`syncai_localizer`, in
# SyncAI-Robot-Workspace) on the relative topic `localization_status`, in the
# bare robot_id namespace, so it lands on <robot_id>/localization_status.
# RELIABLE + TRANSIENT_LOCAL depth 1 -- MappingStatus's QoS and for its reason:
# a reader that starts after the last change learns the state at once. The
# reader must request TRANSIENT_LOCAL too, or nothing is replayed. Published on
# every change of a field below, never periodically.
#
# One consumer: syncai_robot_state, which owns the robot's state machine
# (RobotStatus) and reads two things off this message -- whether an operator
# may confirm the position (`converged`) and whether a confirmation it holds
# still describes the estimate (`pose_epoch`). The localizer holds no
# "confirmed" flag and judges nothing: whether a task may start is
# syncai_robot_state's call, because a task never touches the localizer.
#
# Until 2026-11 none of this left the node. The only surface was the
# relocalize_check service (IsValid), a poll, and a poll at any rate misses a
# guess that is queued and converges between two asks -- which is the event a
# confirmation has to be withdrawn on.

# localizer_node's clock when this sample was published.
builtin_interfaces/Time stamp

# The loaded map's name: the directory holding its map.pcd (`map/<name>/`).
# Empty when the localizer runs with no map (the first deployment's INI), a
# state in which `converged` is always false.
string map

# What relocalize_check answers for code 0: the guess queued last has
# converged. False from the moment a guess is queued (the boot guess,
# relocalize, initialpose, auto_localize, reset_odometry) until a round accepts
# it, and it stays false when the guess is withdrawn on its timeout.
#
# NOT a running quality signal: once true it stays true through a lost or
# degraded stretch, until the next guess. It answers "did the last guess
# land", the question an operator's confirmation is gated on.
bool converged

# Changes whenever the estimate stops being the one a confirmation was given
# for: a map is loaded, or a guess is queued. Compare for EQUALITY only -- the
# value is seeded from the wall clock when the node starts and counts up from
# there, so a restarted localizer never repeats a value an earlier run
# published (a counter from zero would: boot, load, boot guess reaches the
# same number every run, and a confirmation would survive the restart).
uint64 pose_epoch
