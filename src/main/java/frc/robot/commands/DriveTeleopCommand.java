package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

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
                0.05, 0.0, 0.0,
                new TrapezoidProfile.Constraints(
                        swerve.getKinematicLimits().kMaxSteeringVelocity,
                        swerve.getKinematicLimits().kMaxSteeringVelocity * 2));
        snapRotationController.enableContinuousInput(0, 360.0);
        snapRotationController.setTolerance(1.0);
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        if (snapRotationSupplier.get() == null) {
            swerve.drive(translationSupplier.get(), rotationSupplier.get(), fieldOrientedSupplier.get(), false);
            return;
        }

        if(!inSnapRotation) {
            snapRotationController.reset(
                    swerve.getLocalizer().getLatestPose().getRotation().getDegrees(),
                    swerve.getYawVelocity());
            snapRotationController.setConstraints(new TrapezoidProfile.Constraints(
                    swerve.getKinematicLimits().kMaxSteeringVelocity,
                    swerve.getKinematicLimits().kMaxSteeringVelocity * 2));
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

    @Override
    public boolean isFinished() {
        return false;
    }
}
