package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.states.Lights;
import frc.robot.states.TimedIndicatorState;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indicator;

public class DefaultIndicatorCommand extends CommandBase {
    Indicator indicator;
    Indexer indexer;
    public DefaultIndicatorCommand(Indicator indicator, Indexer indexer) {
        this.indicator = indicator;
        this.indexer = indexer;
        addRequirements(indicator);
    }

    @Override
    public void execute() {
        TimedIndicatorState indicatorState;
        if(RobotState.isDisabled()) {
            if(RobotController.getBatteryVoltage() <= 11.0) {
                indicatorState = Lights.LOW_BATTERY;
            } else {
                switch (DriverStation.getAlliance()) {
                    case Red:
                        indicatorState = Lights.RED_ALLIANCE;
                        break;
                    case Blue:
                        indicatorState = Lights.BLUE_ALLIANCE;
                        break;
                    case Invalid:
                    default:
                        indicatorState = Lights.WAITING;
                }
            }
        } else if (RobotState.isEnabled()) {
            if(indexer.getState() == Indexer.State.EJECTING) {
                indicatorState = Lights.PROCESSING_WRONG_CARGO;
            } else {
                indicatorState = Lights.NORMAL;
            }
        } else {
            indicatorState = Lights.NORMAL;
        }

        indicator.setIndicatorState(indicatorState);
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
