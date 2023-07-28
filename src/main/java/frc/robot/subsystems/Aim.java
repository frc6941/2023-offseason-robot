package frc.robot.subsystems;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.vision.GoalTracker;
import com.team254.lib.vision.GoalTracker.TrackReportComparator;
import com.team254.lib.vision.TargetInfo;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.states.AimingParameters;
import lombok.Synchronized;
import org.frcteam6941.localization.Localizer;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.utils.GeometryAdapter;

import java.util.List;
import java.util.Optional;

public class Aim implements Updatable {
    private final GoalTracker goalTracker = new GoalTracker();
    private final Localizer localizer = Swerve.getInstance().getLocalizer();
    private Pose2d cameraToGoal = new Pose2d();

    private static Aim instance;

    public static Aim getInstance() {
        if (instance == null) {
            instance = new Aim();
        }
        return instance;
    }

    private Aim() {

    }

    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> observations) {
        if (observations == null || observations.isEmpty()) {
            goalTracker.maybePruneTracks();
            return;
        }
        updateGoalTracker(timestamp, observations.get(0));
    }

    private com.team254.lib.geometry.Translation2d getCameraTranslationToTarget(TargetInfo target) {
        final double heightDifference = FieldConstants.visionTargetHeightCenter
                - Constants.VisionConstants.HEIGHT_METERS.get();
        final Rotation2d pitch = Rotation2d.fromDegrees(Constants.VisionConstants.PITCH_DEGREES.get());

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

    private final double[] kPossibleTargetNormals = {0.0, 90.0, 180.0, 270.0};

    private void updateGoalTracker(double time, TargetInfo observation) {
        Translation2d cameraToVisionTargetTranslations = getCameraTranslationToTarget(observation);

        Pose2d cameraToVisionTarget = com.team254.lib.geometry.Pose2d.fromTranslation(cameraToVisionTargetTranslations).rotateBy(Rotation2d.kPi);
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
    public synchronized Optional<AimingParameters> getAimingParameters(int previousId) {
        List<GoalTracker.TrackReport> reports = goalTracker.getTracks();
        if (reports.isEmpty()) {
            return Optional.empty();
        }

        double time = Timer.getFPGATimestamp();

        // Find the best track.
        TrackReportComparator comparator = new TrackReportComparator(
                0.0, 10.0, 0.25,
                previousId,
                time
        );

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
        edu.wpi.first.math.geometry.Pose2d poseAtTime = localizer.getCoarseFieldPose(time);
        Pose2d vehicleToGoal = GeometryAdapter.to254(poseAtTime).inverse().transformBy(GeometryAdapter.to254(FieldConstants.hubPose));
        vehicleToGoal = new Pose2d(vehicleToGoal.getTranslation().rotateBy(GeometryAdapter.to254(poseAtTime).getRotation()), vehicleToGoal.getRotation());

        AimingParameters params = new AimingParameters(
                GeometryAdapter.toWpi(vehicleToGoal),
                GeometryAdapter.toWpi(new Pose2d()),
                localizer.getSmoothedVelocity(),
                FieldConstants.hubPose,
                0.0
        );
        return params;
    }

    public void resetVision() {
        goalTracker.reset();
        cameraToGoal = new Pose2d();
    }

    @Override
    public void update(double time, double dt) {
        getAimingParameters(-1).ifPresent(aimingParameters -> {
            localizer.addMeasurement(time, FieldConstants.hubPose.transformBy(
                            new Transform2d(
                                    aimingParameters.getVehicleToTarget().getTranslation(),
                                    aimingParameters.getVehicleToTarget().getRotation()
                            )
                    ),
                    new edu.wpi.first.math.geometry.Pose2d(
                            new edu.wpi.first.math.geometry.Translation2d(0.001, 0.001),
                            new edu.wpi.first.math.geometry.Rotation2d(0.001)));
        });
    }
}
