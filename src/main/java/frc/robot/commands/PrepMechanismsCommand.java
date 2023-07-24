package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.FieldConstants;
import frc.robot.display.ShootingParametersTable;
import frc.robot.states.ShootingParameters;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class PrepMechanismsCommand extends FunctionalCommand {
    public PrepMechanismsCommand(Shooter shooter, Hood hood, Pose2d endPose) {
        super(
                () -> {
                    ShootingParameters preTarget = ShootingParametersTable.getInstance().getParameters(
                            FieldConstants.hubPose.minus(endPose).getTranslation().getNorm()
                    );
                    shooter.setShooterRPM(preTarget.getVelocityRpm());
                    hood.setHoodAngle(preTarget.getBackboardAngleDegree());
                },
                () -> {

                },
                (interrupted) -> {

                },
                () -> {
                    return true;
                }
        );
        addRequirements(shooter, hood);
    }
}
