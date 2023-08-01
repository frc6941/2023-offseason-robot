package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

public class DriveTeleopDoubleCommand extends CommandBase {
    private final Swerve swerve;
    private final Supplier<Translation2d> driverTranslationSupplier;
    private final Supplier<Double> driverRotationSupplier;
    private final Supplier<Translation2d> operatorTranslationSupplier;
    private final Supplier<Double> operatorRotationSupplier;
    private final Supplier<Boolean> fieldOrientedSupplier;

    private final Supplier<Double> snapRotationSupplier;
    private final ProfiledPIDController snapRotationController;
    private boolean inSnapRotation = false;

    public DriveTeleopDoubleCommand(
            Swerve swerve,
            Supplier<Translation2d> driverTranslationSupplier,
            Supplier<Double> driverRotationSupplier,
            Supplier<Translation2d> operatorTranslationSupplier,
            Supplier<Double> operatorRotationSupplier,
            Supplier<Boolean> fieldOrientedSupplier,
            Supplier<Double> snapRotationSupplier) {
        this.swerve = swerve;
        this.driverTranslationSupplier = driverTranslationSupplier;
        this.driverRotationSupplier = driverRotationSupplier;
        this.operatorTranslationSupplier = operatorTranslationSupplier;
        this.operatorRotationSupplier = operatorRotationSupplier;
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
            swerve.drive(
                    fieldOrientedSupplier.get() ? driverTranslationSupplier.get() : operatorTranslationSupplier.get(),
                    fieldOrientedSupplier.get() ? driverRotationSupplier.get() : operatorRotationSupplier.get(),
                    fieldOrientedSupplier.get(),
                    false
            );
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

        if(DriverStation.isTeleopEnabled()) {
            swerve.drive(driverTranslationSupplier.get(), rotation, fieldOrientedSupplier.get(), false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopMovement();
    }

}
