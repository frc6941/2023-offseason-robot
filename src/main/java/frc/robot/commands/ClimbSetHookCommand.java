package frc.robot.commands;

import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Climber;

public class ClimbSetHookCommand extends FunctionalCommand {
    public ClimbSetHookCommand(Climber climber, double setpoint) {
        super(
                () -> {
                },
                () -> {
                    climber.setHookAngle(setpoint);
                },
                (interrupted) -> {
                },
                () -> {
                    return Util.epsilonEquals(setpoint, climber.getHookAngle(), 1.0);
                }
        );
    }
}
