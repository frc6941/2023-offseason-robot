package frc.robot.commands;

import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

public class ClimberSetHookOpenLoopCommand extends FunctionalCommand {
    public ClimberSetHookOpenLoopCommand(Climber climber, double setpoint, double percentage) {
        super(
                () -> {
                },
                () -> {
                    climber.setHookPercentage(percentage);
                },
                (interrupted) -> {
                    climber.lockHook();
                },
                () -> {
                    return Util.epsilonEquals(setpoint, climber.getHookAngle(), 0.5);
                }
        );
        //System.out.println(Constants.ClimberConstants.AutoClimbSetpoints.HOOK_READY_ANGLE);
    }
}
