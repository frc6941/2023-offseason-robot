package frc.robot.commands;

import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Climber;

public class ClimbSetPusherCommand extends FunctionalCommand {
    public ClimbSetPusherCommand(Climber climber, double setpoint) {
        super(
                () -> {
                },
                () -> {
                    climber.setPusherAngle(setpoint);
                },
                (interrupted) -> {
                },
                () -> {
                    return Util.epsilonEquals(setpoint, climber.getPusherAngle(), 1.0);
                }
        );
    }
}
