package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldConstants;
import frc.robot.display.ShootingParametersTable;
import frc.robot.states.ShootingParameters;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class PreaimCommand extends InstantCommand {
    public PreaimCommand(Swerve swerve, Shooter shooter, Hood hood) {
        super(
                () -> {
                    swerve.getTrajectoryFollower().getCurrentTrajectory().ifPresent(trajectory -> {
                        double distance = FieldConstants.hubPose.getTranslation().minus(
                                trajectory.getEndState().poseMeters.getTranslation()
                        ).getNorm();
                        ShootingParameters parameters = ShootingParametersTable.getInstance().getParameters(distance);

                        shooter.setShooterRPM(parameters.getVelocityRpm());
                        hood.setHoodAngle(parameters.getBackboardAngleDegree());
                    });
                }
        );
    }
}
