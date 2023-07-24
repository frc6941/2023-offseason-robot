package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intaker;

public class IntakeDeployCommand extends InstantCommand {
    public IntakeDeployCommand(Intaker intaker) {
        super(() -> {
            intaker.deploy();
            intaker.roll(
                    Constants.IntakerConstants.ROLLING_VOLTAGE.get(),
                    Constants.IntakerConstants.HOPPER_VOLTAGE.get()
            );
        });
        addRequirements(intaker);
    }
}
