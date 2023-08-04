package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

public class DriveToPoseCommand extends CommandBase {
    private final Swerve drivebase;

    // Pose Assist Controller
    private final ProfiledPIDController driveController = new ProfiledPIDController(
            3.0, 0.001, 0,
            Constants.SwerveConstants.TRANSLATION_CONTROLLER_CONSTRAINT
    );

    private final Supplier<Pose2d> targetPose;

    public DriveToPoseCommand(Swerve drivebase, Supplier<Pose2d> targetPose) {
        this.drivebase = drivebase;
        this.targetPose = targetPose;
        driveController.setIntegratorRange(-0.5, 0.5);
        addRequirements(drivebase);
    }

    public DriveToPoseCommand(Swerve drivebase, Pose2d targetPose) {
        this(drivebase, () -> targetPose);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drivebase.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp());
        Pose2d currentVelocity = drivebase.getLocalizer().getMeasuredVelocity();
        Translation2d deltaTranslation = targetPose.get().getTranslation().minus(currentPose.getTranslation());
        double dot = currentVelocity.getX() * deltaTranslation.getX() + currentVelocity.getY() * deltaTranslation.getY();
        driveController.reset(
                deltaTranslation.getNorm(),
                dot / deltaTranslation.getNorm()
        );
        drivebase.resetHeadingController();
        drivebase.setLockHeading(false);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivebase.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp());

        Translation2d deltaTranslation = currentPose.getTranslation().minus(targetPose.get().getTranslation());
        double driveGain = driveController.calculate(deltaTranslation.getNorm(), 0.0);
        Rotation2d angle = new Rotation2d(deltaTranslation.getX(), deltaTranslation.getY());
        Translation2d velocity = new Translation2d(driveGain, angle);

        drivebase.setLockHeading(true);
        drivebase.setHeadingTarget(targetPose.get().getRotation().getDegrees());
        drivebase.drive(velocity, 0.0, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivebase.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp());
        Transform2d delta = targetPose.get().minus(currentPose);
        return delta.getTranslation().getNorm() < 0.05 && Math.abs(delta.getRotation().getDegrees()) < 1.0;
    }
}