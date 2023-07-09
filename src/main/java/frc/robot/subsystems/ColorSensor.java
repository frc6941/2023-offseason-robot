package frc.robot.subsystems;

import org.frcteam6941.drivers.PicoColorSensor;
import org.frcteam6941.looper.Updatable;

import com.team254.lib.util.Util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import lombok.Getter;

public class ColorSensor implements Subsystem, Updatable {
    private static ColorSensor instance;
    private final PicoColorSensor picoColorSensor;
    private boolean sawBall;

    public ColorChoices allianceColor = ColorChoices.NONE;
    public ColorChoices matchedColor;

    public enum ColorChoices {
        RED, BLUE, OTHER, NONE
    }

    @Getter
    private double timestamp;
    @Getter
    private PicoColorSensor.RawColor rawColor = new PicoColorSensor.RawColor();
    @Getter
    private double colorOffset = 0.0; // offset of blue - red
    @Getter
    private double adjustedRed = 0.0;
    @Getter
    private double adjustedBlue = 0.0;
    @Getter
    private double colorRatio = 0.0; // ratio of red to blue color
    @Getter
    private int proximity = 0;
    @Getter
    private boolean sensorConnected = false;


    private final NetworkTableEntry rawColorEntry;
    private final NetworkTableEntry colorOffsetEntry;
    private final NetworkTableEntry adjustedRedEntry;
    private final NetworkTableEntry adjustedBlueEntry;
    private final NetworkTableEntry colorRatioEntry;
    private final NetworkTableEntry proximityEntry;
    private final NetworkTableEntry sensorConnectedEntry;
    

    private ColorSensor() {
        matchedColor = ColorChoices.NONE;
        picoColorSensor = new PicoColorSensor();

        updateMatchedColor();
        updateColorOffset();

        if(Constants.TUNING) {
            ShuffleboardTab dataTab = Shuffleboard.getTab("Color Sensor");
            rawColorEntry = dataTab.add("Raw Color", new double[] {rawColor.red, rawColor.green, rawColor.blue}).getEntry();
            colorOffsetEntry = dataTab.add("Color Offset", colorOffset).getEntry();
            adjustedRedEntry = dataTab.add("Adj Red", adjustedRed).getEntry();
            adjustedBlueEntry = dataTab.add("Adj Blue", adjustedBlue).getEntry();
            colorRatioEntry = dataTab.add("Color Ratio", colorRatio).getEntry();
            proximityEntry = dataTab.add("Proximity", proximity).getEntry();
            sensorConnectedEntry = dataTab.add("Sensor Connected", sensorConnected).getEntry();
        }
    }

    public static ColorSensor getInstance() {
        if (instance == null) {
            instance = new ColorSensor();
        }
        return instance;
    }

    // check if we see a ball
    public boolean seesBall() {
        return !Util.epsilonEquals(colorRatio, 1.0, Constants.ColorSensorConstants.COLOR_SENSOR_RATIO_THRESHOLD);
    }

    // check if we have a new ball
    public boolean seesNewBall() {
        boolean newBall = seesBall() && !sawBall;
        sawBall = seesBall();
        return newBall;
    }

    // check if we have the right color
    public boolean hasCorrectColor() {
        return matchedColor == allianceColor;
    }

    // check if we have the opposite color
    public boolean hasOppositeColor() {
        return !hasCorrectColor()
                && (matchedColor != ColorChoices.OTHER)
                && (matchedColor != ColorChoices.NONE);
    }

    // update baseline color values
    public void updateColorOffset() {
        if (!Double.isNaN(colorRatio)) {
            colorOffset = rawColor.blue - rawColor.red;
        }
    }

    // update our alliance color
    // only should be updated in disabled periodic
    public void updateAllianceColor() {
        if (DriverStation.isDSAttached()) {
            if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                allianceColor = ColorChoices.RED;
            } else if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == DriverStation.Alliance.Blue){
                allianceColor = ColorChoices.BLUE;
            }
        } else {
            allianceColor = ColorChoices.NONE;
            DriverStation.reportError("No Alliance Color Detected", true);
        }
    }

    // update the color of the cargo we see
    public void updateMatchedColor() {
        if (
            Util.epsilonEquals(
                    colorRatio,
                    1.0,
                    Constants.ColorSensorConstants.COLOR_SENSOR_RATIO_THRESHOLD)
        ) {
            matchedColor = ColorChoices.NONE;
        } else {
            if (colorRatio > 1.0) {
                matchedColor = ColorChoices.RED;
            } else if (colorRatio < 1.0) {
                matchedColor = ColorChoices.BLUE;
            } else {
                matchedColor = ColorChoices.OTHER;
            }
        }
    }

    @Override
    public void read(double time, double dt) {
        sensorConnected = picoColorSensor.isSensor0Connected();
        rawColor = picoColorSensor.getRawColor0();
        adjustedBlue = rawColor.blue;
        adjustedRed = rawColor.red + colorOffset;
        colorRatio = adjustedRed / adjustedBlue;
        proximity = picoColorSensor.getProximity0();
        timestamp = picoColorSensor.getLastReadTimestampSeconds();

        updateMatchedColor();
        if(RobotState.isDisabled()) {
            updateColorOffset();
        }
    }

    @Override
    public void telemetry() {
        if(Constants.TUNING) {
            rawColorEntry.setDoubleArray(new double[] {rawColor.red, rawColor.green, rawColor.blue});

            colorOffsetEntry.setDouble(colorOffset);
            adjustedRedEntry.setDouble(adjustedRed);
            adjustedBlueEntry.setDouble(adjustedBlue);
            colorRatioEntry.setDouble(colorRatio);
            proximityEntry.setDouble(proximity);
            sensorConnectedEntry.setBoolean(sensorConnected);
        }
    }
}
