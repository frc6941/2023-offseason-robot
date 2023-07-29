package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

public class DriveTeleopCommand extends CommandBase {
    private final Swerve swerve;
    private final Supplier<Translation2d> translationSupplier;
    private final Supplier<Double> rotationSupplier;
    private final Supplier<Boolean> fieldOrientedSupplier;

    private final Supplier<Double> snapRotationSupplier;
    private final ProfiledPIDController snapRotationController;
    private boolean inSnapRotation = false;

    public DriveTeleopCommand(
            Swerve swerve,
            Supplier<Translation2d> translationSupplier,
            Supplier<Double> rotationSupplier,
            Supplier<Boolean> fieldOrientedSupplier,
            Supplier<Double> snapRotationSupplier) {
        this.swerve = swerve;
        this.translationSupplier = translationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;
        this.snapRotationSupplier = snapRotationSupplier;

        snapRotationController = new ProfiledPIDController(
                0.012, 0.0, 0.00,
                new TrapezoidProfile.Constraints(
                        Constants.SwerveConstants.DRIVETRAIN_MAX_ROTATION_VELOCITY,
                        Constants.SwerveConstants.DRIVETRAIN_MAX_ROTATION_ACCELERATION));
        snapRotationController.enableContinuousInput(0, 360.0);
        snapRotationController.setTolerance(1.0);
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        if (snapRotationSupplier == null || snapRotationSupplier.get() == null) {
            inSnapRotation = false;
            swerve.drive(translationSupplier.get(), rotationSupplier.get(), fieldOrientedSupplier.get(), false);
            return;
        }

        if (!inSnapRotation) {
            snapRotationController.reset(
                    swerve.getLocalizer().getLatestPose().getRotation().getDegrees(),
                    swerve.getYawVelocity());
            snapRotationController.setConstraints(new TrapezoidProfile.Constraints(
                    600,
                    900));
            inSnapRotation = true;
        }

        double rotation = snapRotationController.calculate(
                swerve
                        .getLocalizer()
                        .getLatestPose()
                        .getRotation()
                        .getDegrees(),
                snapRotationSupplier.get()
        );

        swerve.drive(translationSupplier.get(), rotation, fieldOrientedSupplier.get(), false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopMovement();
    }

}
