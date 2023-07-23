package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indicator;

public class LockClimber extends FunctionalCommand {
    public LockClimber(Climber climber, Indicator indicator) {
        super(
                () -> {
                    climber.coastClimber();
                    indicator.turnOn();
                },
                () -> {},
                (interrupted) -> {
                    climber.brakeClimber();
                },
                DriverStation::isEnabled
        );
    }
}
