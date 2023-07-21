package frc.robot.states;

import edu.wpi.first.wpilibj.util.Color;

public class Lights {
    public static final IndicatorState OFF = new IndicatorState(0.0, 0.0, 0.0);
    public static final IndicatorState ALLIANCE_RED = IndicatorState.createFromColor(Color.kFirstRed);
    public static final IndicatorState ALLIANCE_BLUE = IndicatorState.createFromColor(Color.kBlue);
    public static final IndicatorState IP_BLUE = new IndicatorState(97.0 / 255.0, 196.0 / 255.0, 227.0 / 255.0);
    public static final IndicatorState RED = IndicatorState.createFromColor(Color.kRed);
    public static final IndicatorState BLUE = IndicatorState.createFromColor(Color.kBlue);
    public static final IndicatorState GREEN = IndicatorState.createFromColor(Color.kGreen);
    public static final IndicatorState ORANGE = IndicatorState.createFromColor(Color.kOrange);
    public static final IndicatorState WHITE = IndicatorState.createFromColor(Color.kWhite);
    public static final IndicatorState YELLOW = IndicatorState.createFromColor(Color.kYellow);

    // Pit State
    public static final TimedIndicatorState.StaticIndicatorState DARK = new TimedIndicatorState.StaticIndicatorState(OFF);
    public static final TimedIndicatorState.IntervalBlinkingIndicatorState WAITING = new TimedIndicatorState.IntervalBlinkingIndicatorState(RED, ORANGE, OFF, 0.5, 2, 1.0);
    public static final TimedIndicatorState.BreathingIndicatorState RED_ALLIANCE = new TimedIndicatorState.BreathingIndicatorState(ALLIANCE_RED, 5.0);
    public static final TimedIndicatorState.BreathingIndicatorState BLUE_ALLIANCE = new TimedIndicatorState.BreathingIndicatorState(ALLIANCE_BLUE, 5.0);
    public static final TimedIndicatorState.IntervalBlinkingIndicatorState LOW_BATTERY = new TimedIndicatorState.IntervalBlinkingIndicatorState(RED, OFF, OFF, 0.1, 3, 1.0);

    // Chasing State
    public static final TimedIndicatorState.StaticIndicatorState NORMAL = new TimedIndicatorState.StaticIndicatorState(BLUE);
    public static final TimedIndicatorState.BlinkingIndicatorState PROCESSING_WRONG_CARGO = new TimedIndicatorState.BlinkingIndicatorState(RED, OFF, 0.05);

    // Shooting State
    public static final TimedIndicatorState.BlinkingIndicatorState AIMING = new TimedIndicatorState.BlinkingIndicatorState(YELLOW, OFF, 0.05);
    public static final TimedIndicatorState.BlinkingIndicatorState SHOOTING = new TimedIndicatorState.BlinkingIndicatorState(GREEN, OFF, 0.05);
    public static final TimedIndicatorState.BlinkingIndicatorState FENDER = new TimedIndicatorState.BlinkingIndicatorState(YELLOW, OFF, 0.05);

    // Climb State
    public static final TimedIndicatorState.BlinkingIndicatorState ENTER_CLIMB_MODE = new TimedIndicatorState.BlinkingIndicatorState(WHITE, OFF, 0.5);
    public static final TimedIndicatorState.BlinkingIndicatorState WAITING_CONFIRMATION = new TimedIndicatorState.BlinkingIndicatorState(WHITE, YELLOW, 0.05);
    public static final TimedIndicatorState.BlinkingIndicatorState CLIMBING = new TimedIndicatorState.BlinkingIndicatorState(WHITE, GREEN, 0.05);
    public static final TimedIndicatorState.RainbowIndicatorState FINISHED = new TimedIndicatorState.RainbowIndicatorState(5.0);
    public static final TimedIndicatorState.BlinkingIndicatorState MANUAL_CLIMBING = new TimedIndicatorState.BlinkingIndicatorState(WHITE, RED, 0.05);
}
