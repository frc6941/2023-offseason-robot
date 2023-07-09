package frc.robot.states;

import edu.wpi.first.wpilibj.util.Color;

/**
 * From Team 254.
 */
public class IndicatorState {
    public IndicatorState(double r, double g, double b) {
        blue = b;
        green = g;
        red = r;
    }

    public void copyFrom(IndicatorState other) {
        this.blue = other.blue;
        this.green = other.green;
        this.red = other.red;
    }

    public double blue;
    public double green;
    public double red;

    public static IndicatorState createFromHSV(double h, double s, double v) {
        Color converted = Color.fromHSV((int) h, (int) s, (int) v);
        return IndicatorState.createFromColor(converted);
    }

    public static IndicatorState createFromColor(Color color) {
        return new IndicatorState(color.blue, color.green, color.red);
    }
}
