package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.team254.lib.util.Util;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import lombok.Getter;
import org.frcteam6941.drivers.PicoColorSensor;
import org.frcteam6941.looper.Updatable;

public class ColorSensorRio implements Subsystem, Updatable {
    private static ColorSensorRio instance;
    private final ColorSensorV3 revColorSensor;
    private boolean sawBall;

    public ColorChoices allianceColor = ColorChoices.NONE;
    public ColorChoices matchedColor;

    public enum ColorChoices {
        RED, BLUE, OTHER, NONE
    }

    @Getter
    private double timestamp;
    @Getter
    private ColorSensorV3.RawColor rawColor = new ColorSensorV3.RawColor(0,0,0,0);
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


    private ColorSensorRio() {
        matchedColor = ColorChoices.NONE;
        revColorSensor = new ColorSensorV3(I2C.Port.kMXP);

        updateMatchedColor();
        updateColorOffset();

        if(Constants.TUNING) {
            ShuffleboardTab dataTab = Shuffleboard.getTab("Color Sensor Rio");
            rawColorEntry = dataTab.add("Raw Color", new double[] {rawColor.red, rawColor.green, rawColor.blue}).getEntry();
            colorOffsetEntry = dataTab.add("Color Offset", colorOffset).getEntry();
            adjustedRedEntry = dataTab.add("Adj Red", adjustedRed).getEntry();
            adjustedBlueEntry = dataTab.add("Adj Blue", adjustedBlue).getEntry();
            colorRatioEntry = dataTab.add("Color Ratio", colorRatio).getEntry();
            proximityEntry = dataTab.add("Proximity", proximity).getEntry();
            sensorConnectedEntry = dataTab.add("Sensor Connected", sensorConnected).getEntry();
        }
    }

    public static ColorSensorRio getInstance() {
        if (instance == null) {
            instance = new ColorSensorRio();
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
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                allianceColor = ColorChoices.RED;
            } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue){
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
        sensorConnected = revColorSensor.isConnected();
        rawColor = revColorSensor.getRawColor();
        adjustedBlue = rawColor.blue;
        adjustedRed = rawColor.red + colorOffset;
        colorRatio = adjustedRed / adjustedBlue;
        proximity = revColorSensor.getProximity();
        timestamp = 0.0;

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
