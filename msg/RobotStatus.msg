# The robot's state, as RobotState.state carries it. Constants only.
#
# Since 2026-11 this is a state machine syncai_robot_state OWNS -- before, it
# was a derivation (TF present? battery low?) and RUNNING / ERROR were declared
# and never emitted. What moves it:
#
#   boot ──► UNINITIALIZED ──SetRobotState(IDLE)──► IDLE
#   IDLE ──BeginTask accepted──► RUNNING ──EndTask(COMPLETED / CANCELED)──► IDLE
#   RUNNING ──EndTask(FAILED)──► ERROR ──SetRobotState──► IDLE or UNINITIALIZED
#   IDLE ──map switch / new guess / SetRobotState(UNINITIALIZED)──► UNINITIALIZED
#
# RobotState.state is the ROLLUP of that machine and the battery, most severe
# first: ERROR > UNINITIALIZED > RUNNING > CHARGING > WARNING > IDLE.
#
# UNINITIALIZED deliberately takes 0 so that it is also the value a
# default-constructed message carries. Before this constant existed IDLE was 0,
# which meant "nobody has derived a state yet" was indistinguishable from
# "everything is fine, the robot is just idle" — the worst possible default for
# a field consumers use to decide whether to trust the rest of the message.
#
# UNINITIALIZED means NO OPERATOR HAS CONFIRMED THE ROBOT'S POSITION. It is the
# state the robot boots in, and the one it returns to by itself whenever the
# estimate stops being the one that was confirmed: a map is loaded, or a new
# guess is queued (LocalizationStatus.pose_epoch). Only SetRobotState leaves it.
# A boot guess that converges on the last saved pose does NOT -- a robot carried
# to another corridor overnight converges there too, and until 2026-11 a
# schedule drove it off from the wrong place. A task with a MOVE / PATROL /
# CHARGE step is refused here (BeginTask NOT_INITIALIZED); standing up or
# speaking needs no position and is still accepted.
#
# Until 2026-11 it meant only "the map -> base_link TF has never been seen",
# which is over within seconds of boot and never true after a map switch, where
# the localizer keeps broadcasting the previous map's correction. That reading
# survives as RobotState.localization_valid; while that is false the state is
# UNINITIALIZED too.
#
# RUNNING: a task accepted by BeginTask has not been closed by EndTask. One at
# a time -- a second task_id is refused BUSY. A paused task is still RUNNING.
#
# ERROR is LATCHED and has more than one cause by design; RobotState.state_message
# says which. Today the one cause is a task that ended FAILED (EndTask). A fault
# reported by syncai_driver_manager is to be the second. While it holds, EVERY
# task is refused (BeginTask ERROR_LATCHED), and nothing but an operator's
# SetRobotState releases it: a failure nobody has looked at must not be followed
# by the next schedule driving into the same thing. A task refused by BeginTask
# does not latch it, nor does one the operator or the safety lock cancelled.
#
# WARNING means degraded but still operable, and today it has exactly one cause:
# battery below 20%, latched with hysteresis so it does not flap at the publish
# rate. It is advisory — nothing acts on it.
#
# CHARGING (2026-11-18, SyncAI-Robot-State) comes from battery_state's
# power_supply_status, which syncai_driver_manager derives from the sign of the
# pack's NET current with a 0.1 A deadband: the charger delivering more than the
# robot draws. Latched: it holds while a filling pack's current tapers into the
# deadband (NOT_CHARGING) and drops on the first DISCHARGING, which a robot taken
# off its charger reads as soon as it draws its own load. It outranks WARNING (a
# low pack on its charger is what is being fixed) and is outranked by ERROR,
# UNINITIALIZED and RUNNING.
#
# Renumbering was safe when it was done: no consumer in the workspace read the
# numeric value (GET /api/v1/robot/state does not even expose `state`). It does
# mean the `state` numbers inside bags recorded before the change decode to the
# wrong constant.
uint8 UNINITIALIZED = 0
uint8 IDLE          = 1
uint8 RUNNING       = 2
uint8 WARNING       = 3
uint8 ERROR         = 4
uint8 CHARGING      = 5
