package frc.robot.commands;

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
                indicator.setIndicator(Lights.ENTER_CLIMB_MODE),
                new ClimbSetPusherCommand(climber, AutoClimbSetpoints.PUSHER_FORWARD).alongWith(
                        new ClimbSetHookCommand(climber, AutoClimbSetpoints.HOOK_UP)
                ),

                // interval: waiting confirmation for climb auto sequence to start
                indicator.setIndicator(Lights.WAITING_CONFIRMATION),
                new WaitUntilCommand(confirmation::get),
                new InstantCommand(() -> {
                    keepInPlace = true;
                }), // point of no return, keep in place to prevent fall

                // step 2: pull down hook
                indicator.setIndicator(Lights.CLIMBING),
                new ClimbSetHookCommand(climber, AutoClimbSetpoints.HOOK_PULLDOWN),
                // step 3: push backwards
                new ClimbSetPusherCommand(climber, AutoClimbSetpoints.PUSHER_BACK),

                // interval: waiting confirmation for hook release
                indicator.setIndicator(Lights.WAITING_CONFIRMATION),
                new WaitUntilCommand(confirmation::get),

                // step 4: release hook
                new ClimbSetHookCommand(climber, AutoClimbSetpoints.HOOK_RELEASE),
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
