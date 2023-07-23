package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intaker;

public class IntakeRetractCommand extends CommandBase {
    private final Intaker intaker;

    public IntakeRetractCommand(Intaker intaker) {
        this.intaker = intaker;
        addRequirements(intaker);
    }

    @Override
    public void initialize() {
        intaker.retract();
        intaker.stopRolling();
    }
}
