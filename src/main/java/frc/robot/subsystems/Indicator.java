package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Ports;
import frc.robot.display.OperatorDashboard;
import frc.robot.states.IndicatorState;
import frc.robot.states.Lights;
import frc.robot.states.TimedIndicatorState;
import lombok.Getter;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.utils.ColorConversions;

import java.util.Arrays;
import java.util.Map;

public class Indicator implements Updatable, Subsystem {
    private final CANifier ledIndicator = new CANifier(Ports.CanId.Rio.INDICATOR);

    private static Indicator instance;
    private State state = State.ON;
    @Getter
    private final IndicatorState current = new IndicatorState(0, 0, 0);
    private final SuppliedValueWidget<Boolean> colorWidget = Shuffleboard.getTab("MyBot").addBoolean("Color", () -> true);

    public static Indicator getInstance() {
        if (instance == null) {
            instance = new Indicator();
        }
        return instance;
    }

    private Indicator() {

    }

    // Define LED State
    private TimedIndicatorState currentState = Lights.DARK;

    private void setLEDs(IndicatorState color) {
        ledIndicator.setLEDOutput(color.red, CANifier.LEDChannel.LEDChannelB);
        ledIndicator.setLEDOutput(color.green, CANifier.LEDChannel.LEDChannelA);
        ledIndicator.setLEDOutput(color.blue, CANifier.LEDChannel.LEDChannelC);
    }

    public void setIndicatorState(TimedIndicatorState state) {
        this.currentState = state;
    }

    public void turnOff() {
        state = State.OFF;
    }

    public void turnOn() {
        state = State.ON;
    }

    public void abort() { state = State.ABORTED; }

    public Command setIndicator(TimedIndicatorState target) {
        return new InstantCommand(() -> {
            this.setIndicatorState(target);
        }, this);
    }

    public void clearIndicator() {
        if (this.getCurrentCommand() != null) {
            this.getCurrentCommand().cancel();
        }
    }

    @Override
    public void update(double time, double dt) {
        IndicatorState current = new IndicatorState(0, 0, 0);
        this.currentState.getCurrentIndicatorState(current, time);
        switch (state) {
            case ABORTED:
                IndicatorState aborted = new IndicatorState(0, 0, 0);
                Lights.ABORTED.getCurrentIndicatorState(aborted, time);
                this.setLEDs(aborted);
                break;
            case OFF:
                this.setLEDs(new IndicatorState(0, 0, 0));
                break;
            case ON:
                this.setLEDs(current);
                break;
        }
        colorWidget.withProperties(Map.of(
                "colorWhenTrue",
                String.format(
                        "#%02x%02x%02x",
                        (int) current.red * 255, (int) current.green * 255, (int) current.blue * 255
                )
        ));
    }

    public enum State {
        OFF,
        ON,
        ABORTED
    }
}
