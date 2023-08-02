package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.states.Lights;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indicator;

public class DefaultIndicatorCommand extends CommandBase {
    private final Indicator indicator;
    private final Indexer indexer;

    public DefaultIndicatorCommand(Indicator indicator, Indexer indexer) {
        this.indicator = indicator;
        this.indexer = indexer;
        addRequirements(indicator);
    }

    @Override
    public void execute() {
        if (DriverStation.isDisabled()) {
            if (RobotController.getBatteryVoltage() <= 11.8) {
                indicator.setIndicatorState(Lights.LOW_BATTERY);
                return;
            }
            switch (DriverStation.getAlliance()) {
                case Red:
                    indicator.setIndicatorState(Lights.RED_ALLIANCE);
                    return;
                case Blue:
                    indicator.setIndicatorState(Lights.BLUE_ALLIANCE);
                    return;
                case Invalid:
                default:
                    indicator.setIndicatorState(Lights.WAITING);
                    return;
            }
        }

        if (DriverStation.isEnabled()) {
            if(DriverStation.isAutonomousEnabled()) {
                indicator.setIndicatorState(Lights.AUTONOMOUS);
            } else {
                if (indexer.getState() == Indexer.State.EJECTING) {
                    indicator.setIndicatorState(Lights.PROCESSING_WRONG_CARGO);
                } else if (indexer.isWantHold()) {
                    indicator.setIndicatorState(Lights.HOLDING_WRONG_CARGO);
                } else {
                    indicator.setIndicatorState(Lights.NORMAL);
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        indicator.setIndicatorState(Lights.DARK);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
