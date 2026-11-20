Header header
#! control_mode enum msg
#! Depending on the commands message from the task scheduler we receive, the navigation contrllers sets its control mode to be one of these
uint8 NAVI_IDLE=150                   # do not send any commands (useful for landed, taking-off and landing states)
uint8 NAVI_AWAITING_COMMANDS=151      # (UNUSED) (alternative naming CM_READY) I we allow this control_mode (or control_state), then CM_IDLE would ignore commands (hover, waypoint or trajectory commands)
uint8 NAVI_HOVERING=152               # same as CM_WAYPOINT, but the waypoint was set to the drone's current position (in the past)
uint8 NAVI_FLYING_POINT_TO_POINT=153  # go to a single waypoint
uint8 NAVI_WAYPOINT_REACHED=154       # same as CM_WAYPOINT, but the waypoint is the end of a point-point navigation
uint8 NAVI_FLYING_TRAJECTORY=155      # follow a trajectory
uint8 NAVI_TRAJECTORY_FINISHED=156    # same as CM_WAYPOINT, but the waypoint is the last point/end of a trajectory
uint8 NAVI_ABORTED=157                # do not send any commands (the mission has been aborted)
uint8 control_mode
