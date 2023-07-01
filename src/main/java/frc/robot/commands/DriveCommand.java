package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DriveCommand extends CommandBase {
    private final Swerve swerve;
    private final Supplier<Translation2d> translationSupplier;
    private final Supplier<Double> rotationSupplier;
    private final Supplier<Boolean> fieldOrientedSupplier;

    private Supplier<Double> snapRotationSupplier;

    public DriveCommand(
        Swerve swerve,
        Supplier<Translation2d> translationSupplier,
        Supplier<Double> rotationSupplier,
        Supplier<Boolean> fieldOrientedSupplier,

        Supplier<Double> snapRotationSupplier
    ) {
        this.swerve = swerve;
        this.translationSupplier = translationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;
        this.snapRotationSupplier = snapRotationSupplier;
        addRequirements(swerve);
    }
    

    @Override
    public void execute() {
        if(snapRotationSupplier.get() != null) {
            swerve.setLockHeading(true);
            swerve.setHeadingTarget(snapRotationSupplier.get());
        } else {
            swerve.setLockHeading(false);
        }
        swerve.drive(translationSupplier.get(), rotationSupplier.get(), fieldOrientedSupplier.get(), false);
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
