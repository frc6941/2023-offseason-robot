package frc.robot.commands;

import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Climber;

public class ClimberSetPointCommand extends FunctionalCommand {
    public ClimberSetPointCommand(Climber climber, double hookSetpoint, double pusherSetpoint) {
        super(
                () -> {},
                () -> {
                    climber.setHookAngle(hookSetpoint);
                    climber.setPusherAngle(pusherSetpoint);
                },
                (interrupted) -> {
                    climber.lockHook();
                    climber.lockPusher();
                },
                () -> Util.epsilonEquals(hookSetpoint, climber.getHookAngle(), 1.0) && Util.epsilonEquals(pusherSetpoint, climber.getPusherAngle(), 1.0)
        );
        addRequirements(climber);
    }
}
