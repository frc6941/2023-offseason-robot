package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToPoseCommand extends CommandBase {
    Swerve mDrivebase;

    // Pose Assist Controller
    private final ProfiledPIDController driveController = new ProfiledPIDController(
        3.0, 0.001, 0,
        Constants.SwerveConstants.TRANSLATION_CONTROLLER_CONSTRAINT
    );

    private final Supplier<Pose2d> targetPose;

    public DriveToPoseCommand(Swerve mDrivebase, Supplier<Pose2d> targetPose) {
        this.mDrivebase = mDrivebase;
        this.targetPose = targetPose;
        driveController.setIntegratorRange(-0.5, 0.5);
        addRequirements(mDrivebase);
    }

    public DriveToPoseCommand(Swerve mDrivebase, Pose2d targetPose) {
        this(mDrivebase, () -> targetPose);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Pose2d currentVelocity = mDrivebase.getLocalizer().getMeasuredVelocity();
        Translation2d deltaTranslation = targetPose.get().getTranslation().minus(currentPose.getTranslation());
        double dot = currentVelocity.getX() * deltaTranslation.getX() + currentVelocity.getY() * deltaTranslation.getY();
        driveController.reset(
            deltaTranslation.getNorm(),
            dot / deltaTranslation.getNorm()
        );
        mDrivebase.resetHeadingController();
        mDrivebase.setLockHeading(false);
    }

    @Override
    public void execute() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();

        Translation2d deltaTranslation = currentPose.getTranslation().minus(currentPose.getTranslation());
        double driveGain = driveController.calculate(deltaTranslation.getNorm(), 0.0);
        Rotation2d angle = new Rotation2d(deltaTranslation.getX(), deltaTranslation.getY());
        Translation2d velocity = new Translation2d(driveGain, angle);

        mDrivebase.setLockHeading(true);
        mDrivebase.setHeadingTarget(currentPose.getRotation().getDegrees());
        mDrivebase.drive(velocity, 0.0, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Transform2d delta = targetPose.get().minus(currentPose);
        return delta.getTranslation().getNorm() < 0.05 && Math.abs(delta.getRotation().getDegrees()) < 1.0;
    }
}