package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.states.Lights;
import frc.robot.subsystems.ColorSensorRio;
import frc.robot.subsystems.Indicator;

public class ResetColorSensorCommand extends FunctionalCommand {
    public ResetColorSensorCommand(ColorSensorRio colorSensorRio, Indicator indicator) {
        super(
                () -> {},
                () -> {
                    colorSensorRio.updateColorOffset();
                    indicator.setIndicatorState(Lights.COLOR_SENSOR);
                },
                (interrupted) -> {},
                () -> false
        );
        addRequirements(indicator);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
