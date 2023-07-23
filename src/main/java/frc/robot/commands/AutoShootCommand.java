package frc.robot.commands;

import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.JudgeConstants;
import frc.robot.display.OperatorDashboard;
import frc.robot.display.ShootingParametersTable;
import frc.robot.states.AimingParameters;
import frc.robot.states.Lights;
import frc.robot.states.ShootingParameters;
import frc.robot.subsystems.*;
import org.frcteam6328.utils.TunableNumber;
import org.frcteam6941.utils.GeometryAdapter;

import java.util.function.BooleanSupplier;

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

    private final PIDController aimTargetController = new PIDController(0.9, 0.0, 0.0);

    private ShootingParameters parameters;
    private double aimTarget;
    private double aimTargetCompensated;
    private double angularFF;
    private double rangeFF;

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
        isAimed = Util.inRange(new com.team254.lib.geometry.Rotation2d(
                swerve.getLocalizer().getLatestPose().getRotation().minus(
                        Rotation2d.fromDegrees(aimTarget)
                ).getRadians(),
                true
        ).getDegrees(), JudgeConstants.DRIVETRAIN_AIM_TOLERANCE);

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
        aimTarget = new Rotation2d(aimingParameters.getVehicleToTarget().getX(), aimingParameters.getVehicleToTarget().getY()).getDegrees() + 180.0;

       Translation2d velocity_translational = new Translation2d(
               aimingParameters.getVehicleVelocityToField().getX(),
               aimingParameters.getVehicleVelocityToField().getY()
       );
       // Rotate by robot-to-goal rotation; x = radial component (positive towards goal), y = tangential component (positive means turret needs negative lead).
       velocity_translational = velocity_translational.rotateBy(GeometryAdapter.to254(aimingParameters.getVehicleToTarget()).getRotation().inverse());

       double tangential = velocity_translational.y();
       double radial = velocity_translational.x();

       double distance = aimingParameters.getVehicleToTarget().getTranslation().getNorm();
       parameters = parametersTable.getParameters(distance);

       double shotSpeed = distance / flyTime.get() - radial;
       shotSpeed = Util.clamp(shotSpeed, 0, Double.POSITIVE_INFINITY);
       double deltaAdjustment = Units.radiansToDegrees(
               Math.atan2(
                       -tangential, shotSpeed
               )
       );
       aimTarget += deltaAdjustment;

        //parameters = ShootingParametersTable.getInstance().getCustomShotParameters();
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
        if (indexer.getState() == Indexer.State.EJECTING) {
            indicator.setIndicatorState(Lights.PROCESSING_WRONG_CARGO);
        } else if (isAimed && isSpunUp && isHoodUp) {
            indicator.setIndicatorState(Lights.SHOOTING);
        } else {
            indicator.setIndicatorState(Lights.AIMING);
        }
    }

    private void telemetry() {
        OperatorDashboard feedback = OperatorDashboard.getInstance();
        feedback.getReady().setBoolean(isAimed && isSpunUp && isHoodUp);
        feedback.getLockOn().setBoolean(isAimed);
        feedback.getSpunUp().setBoolean(isSpunUp);
        feedback.getHasTarget().setBoolean(Limelight.getInstance().isHasTarget());
    }

    private void clearTelemetry() {
        OperatorDashboard feedback = OperatorDashboard.getInstance();
        feedback.getReady().setBoolean(false);
        feedback.getLockOn().setBoolean(false);
        feedback.getSpunUp().setBoolean(false);
        feedback.getHasTarget().setBoolean(false);
    }


    @Override
    public void initialize() {
        System.out.println("Shoot~!");
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
        //TODO next statement to be deleted 
        indexer.reset();
        hood.setHoodMinimum();
        clearTelemetry();
    }
}
