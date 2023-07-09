package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intaker;

public class RollCommand extends CommandBase {
    private final Intaker intaker;

    public RollCommand(Intaker intaker) {
        this.intaker = intaker;
        addRequirements(this.intaker);
    }

    @Override
    public void execute() {
        intaker.roll();
    }

    @Override
    public void end(boolean interrupted) {
        intaker.stopRolling();
    }
}
