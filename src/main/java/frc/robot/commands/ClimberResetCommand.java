package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimberResetCommand extends CommandBase {
    private final Climber climber;

    public ClimberResetCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.resetHook(0.0);
        climber.resetPusher(0.0);
    }
}
