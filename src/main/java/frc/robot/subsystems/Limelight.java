package frc.robot.subsystems;

import com.team254.lib.geometry.Translation2d;
import com.team254.lib.vision.TargetInfo;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import lombok.Getter;
import org.frcteam6941.looper.Updatable;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Subsystem for interacting with the Limelight 2
 */
public class Limelight implements Updatable, Subsystem {
    private final NetworkTable mNetworkTable;
    private boolean mOutputsHaveChanged = true;

    private final PeriodicIO periodicIO = new PeriodicIO();
    private static final double[] kEmptyDoubleArray = new double[0];
    @Getter
    private boolean hasTarget = false;
    private int mListenerId = -1;

    private static Limelight mInstance;

    public static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }

    private Limelight() {
        mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
        setLed(LedMode.ON);
    }

    private class Listener implements TableEntryListener {
        @Override
        public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
                                 int flags) {
            if (key == "tcornxy") {
                readInputsAndAddVisionUpdate();
                SmartDashboard.putNumber("Last Limelight: ", Timer.getFPGATimestamp());
            }
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;
        public double skew;
        public boolean seesTarget;
        public double shortBoundingBoxLength;
        public double longBoundingBoxLength;
        public double[] corners;

        // OUTPUTS
        public int ledMode = 0; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 0; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    @Override
    public void stop() {
        setLed(LedMode.ON);
    }

    public synchronized void readInputsAndAddVisionUpdate() {
        final double timestamp = Timer.getFPGATimestamp();
        periodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + Constants.VisionConstants.LATENCY;
        periodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        periodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        periodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        periodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        periodicIO.area = mNetworkTable.getEntry("ta").getDouble(0.0);
        periodicIO.shortBoundingBoxLength = mNetworkTable.getEntry("tshort").getDouble(0.0);
        periodicIO.longBoundingBoxLength = mNetworkTable.getEntry("tlong").getDouble(0.0);
        periodicIO.skew = mNetworkTable.getEntry("ts").getDouble(0.0);
        periodicIO.seesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
        periodicIO.corners = mNetworkTable.getEntry("tcornxy").getDoubleArray(kEmptyDoubleArray);

        if (periodicIO.seesTarget) {
            TargetInfo targetInfo = getTarget();
            if (targetInfo != null) {
                Aim.getInstance().addVisionUpdate(
                        timestamp - getLatency(),
                        List.of(targetInfo)
                );
            }
        }
    }

    public boolean targetInfoValid(TargetInfo targetInfo) {
        return true;
    }

    @Override
    public void update(double time, double dt) {
        hasTarget = periodicIO.seesTarget;
        if (periodicIO.givenLedMode != periodicIO.ledMode ||
                periodicIO.givenPipeline != periodicIO.pipeline) {
            //System.out.println("Table has changed from expected, retrigger!!");
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(periodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(periodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(periodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(periodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(periodicIO.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
    public void start() {
        if (mListenerId < 0) {
            mListenerId = mNetworkTable.addEntryListener("tcornxy", new Listener(), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        }
        setLed(LedMode.ON);
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != periodicIO.ledMode) {
            periodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipelineNumber(int mode) {
        if (mode != periodicIO.pipeline) {
//            RobotState.getInstance().resetVision(); // TODO
            periodicIO.pipeline = mode;
            mOutputsHaveChanged = true;
        }
    }

    public double getLatency() {
        return periodicIO.latency;
    }

    public synchronized int getPipeline() {
        return periodicIO.pipeline;
    }

    private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);

    public TargetInfo getTarget() {
        // Get corners
        List<Translation2d> corners = getCorners(periodicIO.corners);

        if (corners.size() < 4) {
            return null;
        }

        // Sort by y, list will have "highest in image" corner first
        corners.sort(ySort);

        // Average the top 4 corners x's
        double sumX = 0;
        int numToAvgX = 4;
        for (int i = 0; i < numToAvgX; i++) {
            sumX += corners.get(i).x();
        }
        double x = sumX / numToAvgX;

        // Average the top 2 corners y's
        double sumY = 0;
        int numToAvgY = 4;
        for (int i = 0; i < numToAvgY; i++) {
            sumY += corners.get(i).y();
        }
        double y = sumY / numToAvgY;

        return getRawTargetInfo(new Translation2d(x, y), Constants.VisionConstants.HORIZONTAL_FOV, Constants.VisionConstants.VERTICAL_FOV);
    }

    public TargetInfo getRawTargetInfo(Translation2d desiredTargetPixel, double kHorizontalFOV, double kVerticalFOV) {
        if (desiredTargetPixel == null) {
            return null;
        } else {
            double VPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
            double VPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));

            double normalizedX = (desiredTargetPixel.x() - Constants.VisionConstants.CAMERA_RESOLUTION[0] * 0.5) /
                    (Constants.VisionConstants.CAMERA_RESOLUTION[0] * 0.5);
            double normalizedY = (Constants.VisionConstants.CAMERA_RESOLUTION[1] * 0.5 - desiredTargetPixel.y()) /
                    (Constants.VisionConstants.CAMERA_RESOLUTION[1] * 0.5);

            double x = -(VPW / 2 * normalizedX); //Negate to Make Left Positive to Match our Frame of Reference
            double y = VPH / 2 * normalizedY;


            return new TargetInfo(x, y);
        }
    }

    private static List<Translation2d> getCorners(double[] tcornxy) {
        // Check if there is a non even number of corners
        if (tcornxy.length % 2 != 0) {
            return List.of();
        }

        ArrayList<Translation2d> corners = new ArrayList<>(tcornxy.length / 2);
        for (int i = 0; i < tcornxy.length; i += 2) {
            corners.add(new Translation2d(tcornxy[i], tcornxy[i + 1]));
        }

        return corners;
    }

    @Override
    public void read(double time, double dt) {
        readInputsAndAddVisionUpdate();
    }

    @Override
    public void telemetry() {
        SmartDashboard.putBoolean("Has Target", periodicIO.seesTarget);
        SmartDashboard.putNumber("Pipeline Latency (ms)", periodicIO.latency);
        SmartDashboard.putNumber("LED Mode", periodicIO.ledMode);
    }

}

