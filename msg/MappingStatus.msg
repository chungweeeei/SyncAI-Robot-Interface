# pgo_node's run state (`syncai_mapping`, in SyncAI-Robot-Workspace).
#
# Published on the relative topic `mapping_status`, so it lands on
# <robot_id>/pgo/mapping_status, RELIABLE + TRANSIENT_LOCAL depth 1 -- the
# same QoS as pgo/map_cloud_file and for the same reason: a backend that
# (re)connects mid-session learns the state at once instead of waiting for
# the next transition. The reader must request TRANSIENT_LOCAL too, or
# nothing is replayed. Published on every transition and at 1 Hz, so a
# consumer can also age it out (the backend treats a sample older than a few
# periods as "unknown", because it outlives the mapping session that
# published it).
#
# The three states and what moves between them:
#
#   IDLE ──start_mapping──► RESETTING ──► MAPPING
#   MAPPING ──reset_mapping──► RESETTING ──► MAPPING   (discard, start over)
#   MAPPING ──save_maps (success)──► IDLE              (the run is finished)
#
# A failed start or reset falls back to the state it left. save_maps and
# reset_mapping are refused in IDLE; start_mapping is refused in MAPPING.
# The .srv files carry the rest of the contract.
#
# IDLE is 0 so a default-constructed message reads as the safe state -- the
# same reasoning RobotStatus gives for UNINITIALIZED = 0.

# No graph, every cloud/odom pair dropped, nothing under /dev/shm. The mapping
# session starts here (unless pgo's start_on_launch parameter says otherwise)
# and a successful save_maps returns here, having freed the keyframes.
uint8 IDLE      = 0
# Keyframes accumulate; the merged map is written to /dev/shm and announced on
# pgo/map_cloud_file. reset_mapping discards the graph and stays here.
uint8 MAPPING   = 1
# Transient (at most a few seconds): a start_mapping or reset_mapping is
# between pausing intake and resuming -- the ResetLIO round trip. Every
# service is refused meanwhile.
uint8 RESETTING = 2

uint8 state

# Keyframes in the graph. 0 outside MAPPING.
uint32 key_poses

# Loop closures the graph has accepted. 0 outside MAPPING.
uint32 loop_closures

# pgo_node's clock when this sample was published. The first interface in this
# package to carry a ROS time rather than a bare uint64 -- see the README's
# gotcha on timestamp units.
builtin_interfaces/Time stamp
