package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intaker;

public class DeployCommand extends CommandBase {
    private final Intaker intaker;

    public DeployCommand(Intaker intaker) {
        this.intaker = intaker;
        addRequirements(this.intaker);
    }

    @Override
    public void execute() {
        intaker.deploy();
    }

    @Override
    public void end(boolean interrupted) {
        intaker.resetDeploy();
    }
}
