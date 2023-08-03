package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ClimberConstants.AutoClimbSetpoints;
import frc.robot.states.Lights;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indicator;
import frc.robot.subsystems.Superstructure;

import java.util.function.Supplier;

public class AutoClimbCommand extends SequentialCommandGroup {
    Climber climber;
    Indicator indicator;
    Supplier<Boolean> confirmation;

    private boolean keepInPlace = false;
    private boolean finished = false;

    public AutoClimbCommand(Climber climber, Indicator indicator, Supplier<Boolean> confirmation) {
        this.climber = climber;
        this.indicator = indicator;
        addCommands(
                // step 1: let hook up to prep climb
                new InstantCommand(() -> {
                    keepInPlace = false;
                    Superstructure.getInstance().inClimb();
                }),
                indicator.setIndicator(Lights.ENTER_CLIMB_MODE),
                new ClimberSetPointCommand(climber, AutoClimbSetpoints.HOOK_READY_ANGLE, AutoClimbSetpoints.PUSHER_START_ANGLE),

                // interval: waiting confirmation for climb auto sequence to start
                indicator.setIndicator(Lights.WAITING_CONFIRMATION),
                new WaitUntilCommand(confirmation::get),
                new InstantCommand(() -> {
                    keepInPlace = true;
                }), // point of no return, keep in place to prevent fall

                // step 2: pull down hook and push backwards
                indicator.setIndicator(Lights.CLIMBING),
                new ClimberSetPointCommand(
                        climber,
                        AutoClimbSetpoints.HOOK_DEMANDED_ANGLE,
                        AutoClimbSetpoints.PUSHER_START_ANGLE
                ).until(() -> climber.getHookAngle() <= AutoClimbSetpoints.HOOK_PUSHER_READY_ANGLE)
                        .andThen(
                                new ClimberSetPointCommand(
                                        climber,
                                        AutoClimbSetpoints.HOOK_DEMANDED_ANGLE,
                                        AutoClimbSetpoints.PUSHER_READY_ANGLE
                                )
                        ),

                // interval: waiting confirmation for push release
                indicator.setIndicator(Lights.WAITING_CONFIRMATION),
                new WaitUntilCommand(confirmation::get),

                // step 3: push go back
                indicator.setIndicator(Lights.CLIMBING),
                new ClimberSetPointCommand(
                        climber,
                        AutoClimbSetpoints.HOOK_DEMANDED_ANGLE,
                        AutoClimbSetpoints.PUSHER_DEMANDED_ANGLE
                ),

                // interval: waiting confirmation for hook release
                indicator.setIndicator(Lights.WAITING_CONFIRMATION),
                new WaitUntilCommand(confirmation::get),
                new WaitCommand( 0.5),

                // step 4: release hook
                indicator.setIndicator(Lights.CLIMBING),
                new ClimberSetHookOpenLoopCommand(climber, AutoClimbSetpoints.HOOK_END_ANGLE, 0.75).until(
                        () -> climber.getHookAngle() > AutoClimbSetpoints.HOOK_END_ANGLE
                ),
                indicator.setIndicator(Lights.FINISHED),
                new InstantCommand(() -> { finished = true; }),
                new WaitUntilCommand(() -> false) // never end the command unless interrupt by abort
        );
    }

    @Override
    public void end(boolean interrupted) {
        Superstructure.getInstance().setStopQueue(false);
        if (keepInPlace) {
            climber.lockPusher();
            climber.lockHook();

            if(finished) {
                indicator.setIndicatorState(Lights.FINISHED);
            } else {
                indicator.abort();
            }
        } else {
            climber.setPusherStart();
            climber.setHookStart();
            climber.setIndexerStart();
            keepInPlace = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
