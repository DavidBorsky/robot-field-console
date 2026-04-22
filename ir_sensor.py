"""IR edge protection helpers.

These sensors hang over the left and right edges of the robot and look for the
carpet boundary so the robot can avoid falling off the playing surface.
"""

from drivetrain import MotorCommand


class IRSensorState:
    def __init__(self, left_edge_detected=False, right_edge_detected=False):
        self.left_edge_detected = left_edge_detected
        self.right_edge_detected = right_edge_detected

    @property
    def any_edge_detected(self) -> bool:
        return self.left_edge_detected or self.right_edge_detected

    @property
    def both_edges_detected(self) -> bool:
        return self.left_edge_detected and self.right_edge_detected


class EdgeCorrection:
    def __init__(self, command, edge_override_active, detail, stop_run=False):
        self.command = command
        self.edge_override_active = edge_override_active
        self.detail = detail
        self.stop_run = stop_run


class EdgeSafetyController:
    """Hard-stop the robot as soon as either down-facing edge sensor trips."""

    def apply(self, requested: MotorCommand, sensors: IRSensorState) -> EdgeCorrection:
        if sensors.any_edge_detected:
            detail = "IR edge detected: motors stopped to prevent leaving the carpet."
            if sensors.both_edges_detected:
                detail = "Both IR edge sensors detected a drop: motors stopped immediately."
            elif sensors.left_edge_detected:
                detail = "Left IR edge sensor detected a drop: motors stopped immediately."
            elif sensors.right_edge_detected:
                detail = "Right IR edge sensor detected a drop: motors stopped immediately."
            return EdgeCorrection(
                command=MotorCommand(front_output=0.0, back_output=0.0),
                edge_override_active=True,
                detail=detail,
                stop_run=True,
            )

        return EdgeCorrection(
            command=requested,
            edge_override_active=False,
            detail="No edge detected: following path normally.",
            stop_run=False,
        )
