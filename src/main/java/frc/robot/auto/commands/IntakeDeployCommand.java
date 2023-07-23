package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intaker;

public class IntakeDeployCommand extends CommandBase {
    private final Intaker intaker;

    public IntakeDeployCommand(Intaker intaker) {
        this.intaker = intaker;
        addRequirements(intaker);
    }

    @Override
    public void initialize() {
        intaker.deploy();
        intaker.roll(
                Constants.IntakerConstants.ROLLING_VOLTAGE.get(),
                Constants.IntakerConstants.HOPPER_VOLTAGE.get()
        );
    }
}
