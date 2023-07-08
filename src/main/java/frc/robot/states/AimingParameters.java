package frc.robot.states;

import edu.wpi.first.math.geometry.Pose2d;
import lombok.AllArgsConstructor;
import lombok.Data;

@Data @AllArgsConstructor
public class AimingParameters {
    private Pose2d vehicleToTarget;
    private Pose2d predictedVehicleToTarget;
    private Pose2d vehicleVelocityToField;
    private Pose2d fieldToTarget;
    private double stability;
}
