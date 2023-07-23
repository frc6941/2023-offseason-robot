package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CargoTracker;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Swerve;

import java.util.Optional;
import java.util.function.BooleanSupplier;

public class AutoAssistedIntakeCommand extends CommandBase {
    private final CargoTracker tracker;
    private final Intaker intaker;
    private final Swerve swerve;
    private final BooleanSupplier stopTracking;

    private DriveToPoseCommand driveCommand;

    public AutoAssistedIntakeCommand(CargoTracker tracker, Intaker intaker, Swerve swerve, BooleanSupplier stopTracking) {
        this.tracker = tracker;
        this.intaker = intaker;
        this.swerve = swerve;
        this.stopTracking = stopTracking;
        addRequirements(tracker, intaker);
    }

    @Override
    public void initialize() {
        intaker.stopRolling();
        intaker.deploy();

    }

    @Override
    public void execute() {
        intaker.deploy();
        if (intaker.isDeployAtSetpoint()) {
            intaker.roll(
                    Constants.IntakerConstants.ROLLING_VOLTAGE.get(),
                    Constants.IntakerConstants.HOPPER_VOLTAGE.get()
            );
        }

        Optional<Transform2d> toCargo = tracker.getCameraToTarget();
        toCargo.ifPresentOrElse(transform2d -> {
            if(!stopTracking.getAsBoolean()) {
                driveCommand = new DriveToPoseCommand(swerve, swerve.getLocalizer().getLatestPose().transformBy(transform2d));
                driveCommand.schedule();
            } else {
                if(driveCommand != null) driveCommand.cancel();
            }
        }, () -> {
            if(driveCommand != null) driveCommand.cancel();
        });
    }

    @Override
    public void end(boolean interrupted) {
        intaker.retract();
        intaker.stopRolling();
        driveCommand.cancel();
    }
}
