package frc.robot.commands;

import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ClimberConstants.AutoClimbSetpoints;
import frc.robot.states.Lights;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indicator;

import java.util.function.Supplier;

public class AutoClimbCommand extends SequentialCommandGroup {
    Climber climber;
    Indicator indicator;
    Supplier<Boolean> confirmation;

    private boolean keepInPlace = false;

    public AutoClimbCommand(Climber climber, Indicator indicator, Supplier<Boolean> confirmation) {
        this.climber = climber;
        this.indicator = indicator;
        addCommands(
                // step 1: let hook up to prep climb
                new InstantCommand(() -> {
                    keepInPlace = false;
                }),
                indicator.setIndicator(Lights.ENTER_CLIMB_MODE),
                new ClimbSetPusherCommand(climber, AutoClimbSetpoints.PUSHER_START_ANGLE).alongWith(
                        new ClimbSetHookCommand(climber, AutoClimbSetpoints.HOOK_READY_ANGLE)
                ),

                // interval: waiting confirmation for climb auto sequence to start
                indicator.setIndicator(Lights.WAITING_CONFIRMATION),
                new WaitUntilCommand(confirmation::get),
                new InstantCommand(() -> {
                    keepInPlace = true;
                }), // point of no return, keep in place to prevent fall

                // step 2: pull down hook and push backwards
                indicator.setIndicator(Lights.CLIMBING),
                new ClimbSetHookCommand(climber, AutoClimbSetpoints.HOOK_DEMANDED_ANGLE).alongWith(
                        new WaitUntilCommand(() -> Util.epsilonEquals(AutoClimbSetpoints.HOOK_PUSHER_READY_ANGLE, climber.getHookAngle(), 1.0))
                                .andThen(new ClimbSetPusherCommand(climber, AutoClimbSetpoints.PUSHER_READY_ANGLE))
                ),
                // step 3: push release a bit
                new ClimbSetPusherCommand(climber, AutoClimbSetpoints.PUSHER_DEMANDED_ANGLE),

                // interval: waiting confirmation for hook release
                indicator.setIndicator(Lights.WAITING_CONFIRMATION),
                new WaitUntilCommand(confirmation::get),

                // step 4: release hook
                new ClimberSetHookOpenLoopCommand(climber, AutoClimbSetpoints.HOOK_READY_ANGLE, 0.1),
                indicator.setIndicator(Lights.FINISHED),
                new WaitUntilCommand(() -> false) // never end the command unless interrupt by abort
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (keepInPlace) {
            climber.lockPusher();
            climber.lockHook();
            indicator.abort();
        } else {
            climber.setPusherMinimum();
            climber.setHookMinimum();
            keepInPlace = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
