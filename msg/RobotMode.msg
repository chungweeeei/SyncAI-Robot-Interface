# Operating modes: one byobu session each, built by syncai_sys_manager's
# NodeManager (switch_mode / get_mode / restart_mode, and RobotState.mode).
#
# MAINTENANCE is what get_mode reports when no session is up; it has no
# session and switch_mode rejects it. CALIBRATION is the lidar-camera
# calibration session (the workspace's config/sessions/calib_lidar_camera.yaml:
# bringup with the livox driver on PointCloud2, plus the camera feeder) -- no
# LIO, no navigation, no driver_manager.
uint8 MAINTENANCE = 0
uint8 MANUAL      = 1
uint8 AUTO        = 2
uint8 CALIBRATION = 3
