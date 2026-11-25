# The robot's aggregated status, published by syncai_robot_state on the relative
# topic `robot_state` (so it lands on <robot_id>/robot_state) — at 1 Hz as
# shipped (params publish_rate: 1.0; the code default is 10 Hz).
#
# One message, one publisher, one consumer: syncai_backend, which re-serialises
# a SUBSET of it for GET /api/v1/robot/state. That REST payload is a frozen
# third-party contract, so the fields below are not all equal in reach — see the
# note above motor_status.
#
# SECONDS since epoch, stamped by the publishing node. Note the unit: the other
# timestamped interface here (ExecuteTask) uses milliseconds.
# Kept in seconds because it is passed through verbatim to
# GET /api/v1/robot/state, whose consumers already treat it as seconds.
#
# At the 1 Hz shipped publish rate it advances by one per message; at the 10 Hz
# code default ten consecutive messages share one timestamp value. Either way it is
# a wall clock for display, not a sequence number: it cannot order samples or
# measure the publish rate -- and motor_status.timestamp below is whole seconds
# too, so it is no help either. Subscribe motor_states directly for sub-second
# resolution; that topic keeps nanoseconds.
uint64 timestamp
string robot_id
string map
# mode values are defined as constants in RobotMode (MAINTENANCE/MANUAL/AUTO) —
# which byobu session is up. NOT the gait controller's own state machine, which is
# `low_level_mode` at the bottom of this message. Three things in this workspace
# are called "mode" and two of them are in here; RobotLowLevelMode.msg carries
# the full disambiguation.
uint8 mode
# state values are defined as constants in RobotStatus
# (UNINITIALIZED/IDLE/RUNNING/WARNING/ERROR/CHARGING). Since 2026-11 this is the
# rollup of a state machine syncai_robot_state OWNS, not a derivation: ERROR
# (a task failed; latched until an operator releases it) > UNINITIALIZED (no
# operator has confirmed the position) > RUNNING (a task accepted by
# begin_task) > CHARGING > WARNING (low battery) > IDLE. RobotStatus has the
# transitions and who drives them.
#
# The order is load-bearing for a reader: a state that is neither ERROR nor
# UNINITIALIZED says the position IS confirmed, which is why no separate
# "confirmed" field exists. The price is that a posture task run on an
# unconfirmed robot reads UNINITIALIZED, not RUNNING, while it runs.
uint8 state
RobotLocalizationStatus localization_status
RobotNetworkStatus network_status
RobotBatteryStatus battery_status

# False when the map -> base_link TF was unavailable at build time. The node
# publishes anyway (it used to drop the whole message, which meant battery, wifi
# and joint temperatures all vanished during exactly the window you most want to
# inspect them: before the localizer has been relocalized). When this is false
# `localization_status` is ZEROED, not held at the last known pose — a stale pose
# with no age attached gets mistaken for a live one, whereas the map origin is an
# obviously suspicious reading.
#
# NOT what `state == UNINITIALIZED` means any more (2026-11). This flag is TF
# presence and nothing else: it turns true a few seconds after boot, when the
# localizer starts broadcasting an identity correction, and reads true against a
# map the robot was never localized in. `state` now waits for an operator's
# confirmation (RobotStatus, SetRobotState). The one direction that still holds:
# while this is false the state is UNINITIALIZED, since there is no pose at all.
#
# syncai_backend gates its RobotRepo write on this flag, which is what keeps
# GET /api/v1/robot/state returning 404 rather than a pose at (0, 0, 0).
bool localization_valid

# Latest per-joint sample, copied from the motor_states topic — the whole
# MotorStates message (only its timestamp is rescaled, see below), so
# `motor_status.states` is the array and
# `motor_status.timestamp` is the sample's own source timestamp.
#
# The two used to be flat siblings here (`MotorState[] motor_status` next to a
# `uint64 motor_timestamp`), which meant this message re-derived a pairing the
# source message already had, and a reader could hold one without the other.
# Nesting MotorStates keeps the array and the instant it describes inseparable,
# and makes the copy in syncai_robot_state a single assignment.
#
# MotorState is reused rather than trimmed: it already carries exactly the fields
# this exists for (temperature, tau_est, error), and q/dq come along for free.
#
# ONLY PARTLY PART OF THE REST PAYLOAD. GET /api/v1/robot/state re-projects each
# entry of `states` through its own RobotMotorStatus model — name / temperature /
# error, the fields that say whether a motor is in trouble. q / dq / ddq / tau_est
# stay internal, and `timestamp` is not exposed at all. The router names its
# fields explicitly and must keep doing so: there is no longer a separate message
# type stopping a widened MotorState from leaking.
#
# Do NOT build motion from q/dq here — this is a 1 Hz (10 Hz at most) diagnostic
# snapshot and `timestamp` at the top of this message has whole-second resolution, so the
# samples cannot even be ordered from it. The high-rate joint channel is the
# backend telemetry WebSocket, which subscribes motor_states directly for that
# reason.
#
# `states` is empty and `timestamp` is 0 until the first motor_states message
# arrives (i.e. whenever syncai_driver_manager is not up). A non-zero timestamp
# that stops advancing is how a consumer tells a fresh reading from one frozen by
# a dead driver_manager.
#
# NOTE THE UNIT, AND THE TRAP IN IT: `motor_status.timestamp` is **SECONDS** here,
# matching `timestamp` at the top of this message — but the SAME MotorStates
# message on the `motor_states` topic carries NANOSECONDS, which is what
# syncai_driver_manager stamps. syncai_robot_state converts on the way in.
#
# So a MotorStates' unit depends on where you found it. Do not compare this value
# against one read off the topic without scaling, and do not "simplify"
# syncai_robot_state's conversion away. Seconds was asked for here because every
# other timestamp this message exposes is in seconds; the topic keeps nanoseconds
# because the backend's telemetry WebSocket is the high-rate joint channel and
# whole seconds cannot order samples at 20 Hz.
MotorStates motor_status

# The gait controller's OWN low-level state machine, copied verbatim from the
# `mode` topic (std_msgs/Int32MultiArray, published by syncai_driver_manager from
# the MODE_STATE telemetry section). NOT `mode` above — see RobotLowLevelMode.msg,
# which carries both integer vocabularies, the known gap in them (MPC), and why
# they are comments rather than constants.
#
# This is the first MEASURED counterpart to a stack full of commanded state:
# SetPolicyMode and SetMotionKey are one-way UDP with no acknowledgement, so until
# now nothing could report what the controller was actually doing.
#
# ALL ZEROES IS AMBIGUOUS. `0 / 0` is what this carries before the first sample
# arrives AND a genuine "PPO / Stand" reading; there is no field here that tells
# the two apart, and no freshness information of any kind — see
# RobotLowLevelMode.msg, which spells out what that costs and why.
#
# syncai_robot_state attaches no staleness verdict, and this field does NOT feed
# `state`: RobotStatus has no STALE value, UNINITIALIZED belongs to localization,
# and WARNING carries no reason field, so a third cause folded into it would be
# unreadable. Same split as localization_valid vs state — the precise field
# answers the precise question, `state` stays the coarse rollup.
#
# EXPOSED over GET /api/v1/robot/state, unlike motor_status.timestamp and
# localization_valid. That was a deliberate decision rather than the default: the
# router's field list is a whitelist, and this is the only way the console can show
# what the controller is doing rather than what it was told to do. The router
# decodes both integers to LABELS ONLY -- the raw values stay here, on this topic,
# which is where you look when a label comes back UNKNOWN (MPC's code, or the
# controller's startup sentinel; REST cannot tell them apart).
RobotLowLevelMode low_level_mode

# The localizer's last guess has converged (LocalizationStatus.converged,
# copied): there is an estimate an operator can confirm, so SetRobotState to
# IDLE will not be refused NOT_CONVERGED. False until the localizer has been
# heard from, and for as long as the mapping session runs without one.
#
# It says the Confirm control may be pressed, NOT that the position is right --
# that judgement is the operator's, looking at the live scan on the map, and it
# is the whole reason a converged boot guess does not confirm itself.
#
# EXPOSED over GET /api/v1/robot/state, deliberately, like low_level_mode: the
# console disables its Confirm control on it.
bool localization_converged

# One English sentence for an operator saying why the state is what it is:
# what the robot is waiting for while UNINITIALIZED, which task is RUNNING,
# which task failed and with what reason while ERROR. Written to be shown
# verbatim, so it names no node, topic or service. Empty when there is nothing
# to add to the state's own name (IDLE, WARNING, CHARGING).
#
# The reason field RobotLowLevelMode's note above says WARNING lacks: ERROR has
# more than one cause by design (a failed task today, a driver fault later), and
# a state with several causes is unreadable without one.
#
# EXPOSED over GET /api/v1/robot/state, deliberately.
string state_message
