package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intaker;

public class IntakeRetractCommand extends InstantCommand {
    public IntakeRetractCommand(Intaker intaker) {
        super(() -> {
            intaker.retract();
            intaker.stopRolling();
        });
        addRequirements(intaker);
    }
}
