package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intaker;

public class AutoIntakeCommand extends CommandBase {
    private final Intaker intaker;

    public AutoIntakeCommand(Intaker intaker) {
        this.intaker = intaker;
        addRequirements(intaker);
    }

    @Override
    public void initialize() {
        intaker.stopRolling();
        intaker.deploy();
    }

    @Override
    public void execute() {
        intaker.deploy();
        intaker.roll(
                Constants.IntakerConstants.ROLLING_VOLTAGE.get(),
                Constants.IntakerConstants.HOPPER_VOLTAGE.get()
        );
    }

    @Override
    public void end(boolean interrupted) {
        intaker.retract();
        intaker.stopRolling();
    }
}
