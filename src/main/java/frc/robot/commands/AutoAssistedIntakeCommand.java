package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CargoTracker;
import frc.robot.subsystems.Intaker;

import java.util.function.DoubleSupplier;

public class AutoAssistedIntakeCommand extends CommandBase {
    private final CargoTracker tracker;
    private final Intaker intaker;

    public AutoAssistedIntakeCommand(CargoTracker tracker, Intaker intaker) {
        this.tracker = tracker;
        this.intaker = intaker;
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
    }

    @Override
    public void end(boolean interrupted) {
        intaker.contract();
        intaker.stopRolling();
    }
}
