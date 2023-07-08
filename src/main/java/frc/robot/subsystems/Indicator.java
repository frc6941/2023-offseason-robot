package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Ports;
import frc.robot.states.IndicatorState;
import frc.robot.states.Lights;
import frc.robot.states.TimedIndicatorState;
import lombok.Getter;
import org.frcteam6941.looper.Updatable;

public class Indicator implements Updatable, Subsystem {
    private final CANifier ledIndicator = new CANifier(Ports.CanId.Rio.INDICATOR);

    private static Indicator instance;
    private State state = State.ON;

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

    public Command setIndicator(TimedIndicatorState target) {
        return new InstantCommand(() -> {
            this.setIndicatorState(target);
        }, this);
    }

    public void clearIndicator() {
        this.getCurrentCommand().cancel();
    }

    @Override
    public void update(double time, double dt) {
        IndicatorState current = new IndicatorState(0, 0, 0);
        this.currentState.getCurrentIndicatorState(current, time);
        switch (state) {
            case OFF:
                this.setLEDs(new IndicatorState(0, 0, 0));
                break;
            case ON:
                this.setLEDs(current);
                break;
        }
        SmartDashboard.putNumberArray("Indicator LED State", new double[]{current.red, current.green, current.blue});
    }

    public enum State {
        OFF,
        ON
    }
}
