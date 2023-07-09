package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.frcteam6941.localization.Localizer;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.utils.GeometryAdapter;
import org.photonvision.targeting.PhotonPipelineResult;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.vision.GoalTracker;
import com.team254.lib.vision.GoalTracker.TrackReportComparator;
import com.team254.lib.vision.TargetInfo;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.states.AimingParameters;
import lombok.Synchronized;

public class Aim implements Updatable {
    public static class PeriodicIO {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;
        public boolean has_comms;
        public boolean sees_target;

        public double dt;

        // OUTPUTS
        public int ledMode = 3; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    public PeriodicIO periodicIO = new PeriodicIO();
    private boolean mOutputsHaveChanged = false;

    private final PhotonPipelineResult latestResult = new PhotonPipelineResult();
    private final List<TargetInfo> targetInfos = new ArrayList<>();

    private int mLatencyCounter = 0;

    // distance to target
    public Optional<Double> distanceToTarget = Optional.empty();
    private final NetworkTable mNetworkTable;
    private final boolean isConnected = false;

    private final GoalTracker goalTracker = new GoalTracker();
    private final Localizer localizer = Swerve.getInstance().getLocalizer();

    private static Aim instance;

    public static Aim getInstance() {
        if (instance == null) {
            instance = new Aim();
        }
        return instance;
    }
    
    private Aim() {
        mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
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

    public synchronized void setPipeline(int mode) {
        if (mode != periodicIO.pipeline) {
            periodicIO.pipeline = mode;

            System.out.println(periodicIO.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    @Synchronized
    public Optional<Double> getLimelightDistanceToTarget() {
        return distanceToTarget;
    }

    @Synchronized
    public boolean isConnected() {
        return isConnected;
    }

    @Synchronized
    public boolean hasTarget() {
        return latestResult.hasTargets();
    }

    private void updateDistanceToTarget() {
        if(latestResult.hasTargets()) {
            double goal_theta = Units.degreesToRadians(Constants.VisionConstants.PITCH_DEGREES)
                    + Math.toRadians(latestResult.getBestTarget().getYaw());
            double height_diff = FieldConstants.visionTargetHeightCenter - Constants.VisionConstants.HEIGHT_METERS;

            distanceToTarget = Optional.of(height_diff / Math.tan(goal_theta) + FieldConstants.visionTargetDiameter * 0.5);
        } else {
            distanceToTarget = Optional.empty();
        }
        
    }

    private com.team254.lib.geometry.Translation2d getCameraTranslationToTarget(TargetInfo target) {
        final double heightDifference = FieldConstants.visionTargetHeightCenter
                - Constants.VisionConstants.HEIGHT_METERS;
        final Rotation2d pitch = Rotation2d.fromDegrees(Constants.VisionConstants.PITCH_DEGREES);

        // Compensate for camera pitch
        Translation2d xzTranslation = new Translation2d(target.getX(), target.getZ()).rotateBy(pitch);
        double x = xzTranslation.x();
        double y = target.getY();
        double z = xzTranslation.y();

        // find intersection with the goal
        if ((z > 0.0) == (heightDifference > 0.0)) {
            double scaling = heightDifference / z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new com.team254.lib.geometry.Translation2d(
                    distance * angle.cos() + FieldConstants.visionTargetDiameter * 0.5, distance * angle.sin());
        }
        return null;
    }

    private final double[] kPossibleTargetNormals = { 0.0, 90.0, 180.0, 270.0 };

    private void updateGoalTracker(double time, List<TargetInfo> observations) {
        List<Translation2d> cameraToVisionTargetTranslations = new ArrayList<>();

        if (observations == null || observations.isEmpty()) {
            goalTracker.maybePruneTracks();
            return;
        }

        for (TargetInfo target : observations) {
            cameraToVisionTargetTranslations.add(getCameraTranslationToTarget(target));
        }

        if (cameraToVisionTargetTranslations.get(0) != null && cameraToVisionTargetTranslations.size() == 1) {
            return;
        }
        Pose2d cameraToVisionTarget = com.team254.lib.geometry.Pose2d
                .fromTranslation(cameraToVisionTargetTranslations.get(0));
        SmartDashboard.putString("Camera To Vision Target", cameraToVisionTarget.toString());
        edu.wpi.first.math.geometry.Pose2d currentPose = localizer.getPoseAtTime(time);
        goalTracker.update(time, List.of(
                new Pose2d(
                        new Pose2d(
                                currentPose.getX(), currentPose.getY(),
                                new Rotation2d(currentPose.getRotation().getRadians(), true))
                                .transformBy(cameraToVisionTarget.inverse()).getTranslation(),
                        Rotation2d.identity())));
    }

    public synchronized Pose2d getFieldToGoal() {
        if (!goalTracker.hasTracks()) {
            return null;
        }

        Pose2d fieldToTarget = goalTracker.getTracks().get(0).field_to_target;

        double normalPositive = (fieldToTarget.getRotation().getDegrees() + 360) % 360;
        double normalClamped = kPossibleTargetNormals[0];
        for (double possible : kPossibleTargetNormals) {
            if (Math.abs(normalPositive - possible) < Math.abs(normalPositive - normalClamped)) {
                normalClamped = possible;
            }
        }

        return new Pose2d(fieldToTarget.getTranslation(), Rotation2d.fromDegrees(normalClamped));
    }

    @Synchronized
    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = new ArrayList<TargetInfo>();
        targets.add(new TargetInfo(Math.tan(Math.toRadians(-periodicIO.xOffset)), Math.tan(Math.toRadians(periodicIO.yOffset))));
        if (hasTarget() && targets != null) {
            return targets;
        }

        return null;
    }

    @Synchronized
    public synchronized Optional<AimingParameters> getAimingParameters(int previousId) {
        List<GoalTracker.TrackReport> reports = goalTracker.getTracks();

        if (reports.isEmpty()) {
            return Optional.empty();
        }

        double time = Timer.getFPGATimestamp();

        // Find the best track.
        TrackReportComparator comparator = new TrackReportComparator(
                0, 10, 100, 
                previousId,
                time);
                
        reports.sort(comparator);

        GoalTracker.TrackReport report = null;
        for (GoalTracker.TrackReport track : reports) {
            if (track.latest_timestamp > time - Constants.VisionConstants.TRACK_MAX_AGE) {
                report = track;
                break;
            }
        }
        
        if (report == null) {
            return Optional.empty();
        }

        edu.wpi.first.math.geometry.Pose2d poseAtTime = localizer.getPoseAtTime(time);
        edu.wpi.first.math.geometry.Pose2d predictedPoseAtTime = localizer.getPredictedPose(0.5);
        Pose2d vehicleToGoal = GeometryAdapter.to254(poseAtTime).inverse().transformBy(report.field_to_target);
        vehicleToGoal = new Pose2d(vehicleToGoal.getTranslation().rotateBy(GeometryAdapter.to254(poseAtTime).getRotation()), vehicleToGoal.getRotation());
        Pose2d predictedVehicleToGoal = GeometryAdapter.to254(predictedPoseAtTime).inverse().transformBy(report.field_to_target);
        predictedVehicleToGoal = new Pose2d(predictedVehicleToGoal.getTranslation().rotateBy(GeometryAdapter.to254(predictedPoseAtTime).getRotation()), predictedVehicleToGoal.getRotation());

        AimingParameters params = new AimingParameters(
            GeometryAdapter.toWpi(vehicleToGoal),
            GeometryAdapter.toWpi(predictedVehicleToGoal),
            localizer.getSmoothedVelocity(),
            GeometryAdapter.toWpi(report.field_to_target),
            report.stability
        );
        return Optional.of(params);
    }

    @Synchronized
    public synchronized AimingParameters getDefaultAimingParameters() {
        double time = Timer.getFPGATimestamp();
        edu.wpi.first.math.geometry.Pose2d poseAtTime = localizer.getPoseAtTime(time);
        edu.wpi.first.math.geometry.Pose2d predictedPoseAtTime = localizer.getPredictedPose(0.5);
        Pose2d vehicleToGoal = GeometryAdapter.to254(poseAtTime).inverse().transformBy(GeometryAdapter.to254(FieldConstants.hubPose));
        vehicleToGoal = new Pose2d(vehicleToGoal.getTranslation().rotateBy(GeometryAdapter.to254(poseAtTime).getRotation()), vehicleToGoal.getRotation());
        Pose2d predictedVehicleToGoal = GeometryAdapter.to254(predictedPoseAtTime).inverse().transformBy(GeometryAdapter.to254(FieldConstants.hubPose));
        predictedVehicleToGoal = new Pose2d(predictedVehicleToGoal.getTranslation().rotateBy(GeometryAdapter.to254(predictedPoseAtTime).getRotation()), predictedVehicleToGoal.getRotation());

        AimingParameters params = new AimingParameters(
            GeometryAdapter.toWpi(vehicleToGoal),
            GeometryAdapter.toWpi(predictedVehicleToGoal),
            localizer.getSmoothedVelocity(),
            FieldConstants.hubPose,
            0.0
        );
        return params;
    }

    @Override
    public void read(double time, double dt) {
        final double latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + Constants.VisionConstants.LATENCY;
        periodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(3.0);
        periodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        periodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        periodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        periodicIO.area = mNetworkTable.getEntry("ta").getDouble(0.0);

        if (latency == periodicIO.latency) {
            mLatencyCounter++;
        } else {
            mLatencyCounter = 0;
        }

        periodicIO.latency = latency;
        periodicIO.has_comms = mLatencyCounter < 10;

        periodicIO.sees_target = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
    }

    @Override
    public void update(double time, double dt) {
        updateDistanceToTarget();
        updateGoalTracker(time, targetInfos);
    }

    @Override
    public void write(double time, double dt) {
        if (periodicIO.givenLedMode != periodicIO.ledMode || periodicIO.givenPipeline != periodicIO.pipeline) {
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
    public void telemetry() {
        SmartDashboard.putBoolean("Limelight Ok", periodicIO.has_comms);
        SmartDashboard.putNumber("limelight" + ": Pipeline Latency (ms)", periodicIO.latency);
        SmartDashboard.putNumber("Limelight dt", periodicIO.dt);

        SmartDashboard.putBoolean("limelight" + ": Has Target", periodicIO.sees_target);
        SmartDashboard.putNumber("Limelight Tx: ", periodicIO.xOffset);
        SmartDashboard.putNumber("Limelight Ty: ", periodicIO.yOffset);

        SmartDashboard.putNumber("Limelight Distance To Target", distanceToTarget.orElse(-1.0));
    }

    @Override
    public void start() {
        setLed(LedMode.ON);
        goalTracker.reset();
    }

    @Override
    public void stop() {
        setLed(LedMode.OFF);
        goalTracker.reset();
    }
}
