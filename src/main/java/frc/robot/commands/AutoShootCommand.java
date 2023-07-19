package frc.robot.commands;

import java.util.function.BooleanSupplier;

import frc.robot.display.OperatorDashboard;
import frc.robot.states.Lights;
import frc.robot.subsystems.*;
import org.frcteam6328.utils.TunableNumber;
import org.frcteam6941.utils.AngleNormalization;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.states.AimingParameters;
import frc.robot.states.ShootingParameters;
import frc.robot.display.ShootingParametersTable;

public class AutoShootCommand extends CommandBase {
    private final Swerve swerve;
    private final Indexer indexer;
    private final Trigger trigger;
    private final Shooter shooter;
    private final Hood hood;
    private final Aim aim;
    private final Indicator indicator;
    private final ShootingParametersTable parametersTable;
    private final BooleanSupplier overrideAim;

    private final TunableNumber flyTime = new TunableNumber("Cargo Fly Time", 1.03);

    private ShootingParameters parameters;
    private double aimTarget;

    private boolean isAimed = false;
    private boolean isSpunUp = false;
    private boolean isHoodUp = false;


    public AutoShootCommand(Swerve swerve, Indexer indexer, Trigger trigger,
                            Shooter shooter, Hood hood, Aim aim, Indicator indicator,
                            ShootingParametersTable parametersTable, BooleanSupplier overrideAim) {
        this.swerve = swerve;
        this.indexer = indexer;
        this.trigger = trigger;
        this.shooter = shooter;
        this.hood = hood;
        this.aim = aim;
        this.indicator = indicator;
        this.parametersTable = parametersTable;
        this.overrideAim = overrideAim;

        addRequirements(trigger, shooter, hood, indicator);
    }


    private void judgeStatus() {
        isAimed = overrideAim.getAsBoolean() || Util.epsilonEquals(
                AngleNormalization.placeInAppropriate0To360Scope(aimTarget, swerve.getLocalizer().getLatestPose().getRotation().getDegrees()),
                aimTarget,
                Constants.JudgeConstants.DRIVETRAIN_AIM_TOLERANCE
        );

        isSpunUp = Util.epsilonEquals(
                shooter.getShooterRPM(),
                parameters.getVelocityRpm(),
                Constants.JudgeConstants.FLYWHEEL_RPM_TOLERANCE
        );

        isHoodUp = hood.isCalibrated() && Util.epsilonEquals(
                hood.getHoodAngle(),
                parameters.getBackboardAngleDegree(),
                Constants.JudgeConstants.BACKBOARD_ANGLE_TOLERANCE
        );
    }

    private void updateShootingParameters() {
        AimingParameters aimingParameters = aim.getAimingParameters(-1).orElse(aim.getDefaultAimingParameters());
        Pose2d predictedVehicleToGoal = aimingParameters.getPredictedVehicleToTarget();
        Pose2d preaimVehicleToGoal = predictedVehicleToGoal.transformBy(
                new Transform2d(
                        aimingParameters.getVehicleVelocityToField().getTranslation().rotateBy(predictedVehicleToGoal.getRotation()),
                        aimingParameters.getVehicleVelocityToField().getRotation()
                ).times(-flyTime.get())
        );
        aimTarget = new Rotation2d(preaimVehicleToGoal.getX(), preaimVehicleToGoal.getY()).plus(Rotation2d.fromDegrees(180.0)).getDegrees();

        double distance = predictedVehicleToGoal.getTranslation().getNorm();
        parameters = parametersTable.getParameters(distance);
    }

    private void setMechanisms() {
        swerve.setLockHeading(!overrideAim.getAsBoolean());
        swerve.setHeadingTarget(aimTarget);
        hood.setHoodAngle(parameters.getBackboardAngleDegree());
        shooter.setShooterRPM(parameters.getVelocityRpm());

        if (isAimed && isSpunUp && isHoodUp) {
            trigger.feed(false);
            indexer.setWantFeed(true);
        } else {
            trigger.lock();
            indexer.setWantFeed(false);
        }
    }

    private void setIndicator() {
        if(indexer.getState() == Indexer.State.EJECTING) {
            indicator.setIndicator(Lights.PROCESSING_WRONG_CARGO).schedule();
        } else if (isAimed && isSpunUp && isHoodUp) {
            indicator.setIndicator(Lights.SHOOTING).schedule();
        } else {
            indicator.setIndicator(Lights.AIMING).schedule();
        }
    }

    private void telemetry() {
        OperatorDashboard feedback = OperatorDashboard.getInstance();
        feedback.getReady().setBoolean(isAimed && isSpunUp && isHoodUp);
        feedback.getLockOn().setBoolean(isAimed);
        feedback.getSpunUp().setBoolean(isSpunUp);
    }

    private void clearTelemetry() {
        OperatorDashboard feedback = OperatorDashboard.getInstance();
        feedback.getReady().setBoolean(false);
        feedback.getLockOn().setBoolean(false);
        feedback.getSpunUp().setBoolean(false);
    }


    @Override
    public void initialize() {
        swerve.setKinematicsLimit(Constants.SwerveConstants.DRIVETRAIN_LIMITED);
        shooter.turnOff();
        trigger.lock();
        indexer.setWantFeed(false);
    }

    @Override
    public void execute() {
        updateShootingParameters();
        judgeStatus();
        setMechanisms();
        setIndicator();
        telemetry();
    }

    @Override
    public void end(boolean isInterrupted) {
        swerve.setLockHeading(false);
        swerve.setKinematicsLimit(Constants.SwerveConstants.DRIVETRAIN_UNCAPPED);
        shooter.idle();
        trigger.lock();
        indexer.setWantFeed(false);
        hood.setHoodMinimum();
        clearTelemetry();
    }
}
