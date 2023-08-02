package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.states.Lights;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indicator;

public class LockClimber extends FunctionalCommand {
    public LockClimber(Climber climber, Indicator indicator) {
        super(
                () -> {
                    climber.coastClimber();
                    indicator.turnOn();
                    indicator.setIndicatorState(Lights.CALIBRATING);
                },
                () -> {},
                (interrupted) -> {
                    climber.brakeClimber();
                    climber.resetHook(0.0);
                    climber.resetPusher(0.0);
                },
                DriverStation::isEnabled
        );
        addRequirements(indicator);
    }
    @Override
    public boolean runsWhenDisabled() {
        // TODO Auto-generated method stub
        return true;
    }
}
