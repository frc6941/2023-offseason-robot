package frc.robot.display;

import org.frcteam6941.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;

public class FieldView {
    private final Field2d mField2d = new Field2d();

    public FieldView() {
        SmartDashboard.putData(mField2d);
    }

    public void update(Pose2d pose, Pose2d ghost, SwerveModuleState[] states, SwerveDrivetrainConstants constants) {
        Pose2d[] mModulePoses = new Pose2d[states.length];
        for (int i = 0; i < states.length; i++) {
            Translation2d updatedPosition = constants.getDrivetrainModPositions()[i].rotateBy(pose.getRotation()).plus(pose.getTranslation());
            Rotation2d updatedRotation = states[i].angle.plus(pose.getRotation());
            if (states[i].speedMetersPerSecond < 0.0) {
                updatedRotation = updatedRotation.plus(Rotation2d.fromDegrees(180));
            }
            mModulePoses[i] = new Pose2d(updatedPosition, updatedRotation);
        }

        mField2d.setRobotPose(pose);
        mField2d.getObject("Ghost").setPose(ghost);
        // mField2d.getObject("Swerve Modules").setPoses(mModulePoses);
        mField2d.getObject("Target").setPose(new Pose2d(FieldConstants.hubCenter, new Rotation2d()));
    }

}
