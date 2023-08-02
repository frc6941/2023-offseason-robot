package frc.robot.commands;

import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.util.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.JudgeConstants;
import frc.robot.FieldConstants;
import frc.robot.display.OperatorDashboard;
import frc.robot.display.ShootingParametersTable;
import frc.robot.states.AimingParameters;
import frc.robot.states.Lights;
import frc.robot.states.ShootingParameters;
import frc.robot.subsystems.*;
import org.frcteam6328.utils.TunableNumber;
import org.frcteam6941.utils.GeometryAdapter;

import java.util.Optional;
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
    private final boolean wait;

    private final boolean moveAndShoot = true;
    private final TunableNumber flyTime = new TunableNumber("Cargo Fly Time", 1.02);


    private final TunableNumber headingKp = new TunableNumber("Heading Kp", 0.015);
    private final TunableNumber headingKi = new TunableNumber("Heading Ki", 0.001);
    private final TunableNumber headingKd = new TunableNumber("Heading Kd", 0.0005);
    private final TunableNumber headingKs = new TunableNumber("Heading Ks", 0.01);
    private final PIDController shootingController = new PIDController(headingKp.get(), headingKi.get(),headingKd.get());
    private final TunableNumber headingLargeKf = new TunableNumber("Heading Large Kf", 0.015);
    private boolean isLarge = false;

    private ShootingParameters parameters;
    private double aimTarget;
    private double angleTolerance;
    private double aimTargetCompensated;

    private final TimeDelayedBoolean aimReady = new TimeDelayedBoolean();

    private boolean isAimed = false;
    private boolean isSpunUp = false;
    private boolean isHoodUp = false;
    private boolean isValid = false;


    public AutoShootCommand(Swerve swerve, Indexer indexer, Trigger trigger,
                            Shooter shooter, Hood hood, Aim aim, Indicator indicator,
                            ShootingParametersTable parametersTable, BooleanSupplier overrideAim, boolean wait) {
        this.swerve = swerve;
        this.indexer = indexer;
        this.trigger = trigger;
        this.shooter = shooter;
        this.hood = hood;
        this.aim = aim;
        this.indicator = indicator;
        this.parametersTable = parametersTable;
        this.overrideAim = overrideAim;
        this.wait = wait;

        shootingController.enableContinuousInput(0, 360.0);
        shootingController.setIntegratorRange(-0.05, 0.05);

        addRequirements(trigger, shooter, hood, indicator);
    }


    private void judgeStatus() {
        isAimed = aimReady.update(
                Util.inRange(new com.team254.lib.geometry.Rotation2d(
                        swerve.getLocalizer().getLatestPose().getRotation().minus(
                                Rotation2d.fromDegrees(aimTarget)
                        ).getRadians(),
                        true
                ).getDegrees(), angleTolerance)&& isValid
                        && Limelight.getInstance().isHasTarget(), 0.1
        ) || overrideAim.getAsBoolean();

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
        Optional<AimingParameters> aimP = aim.getAimingParameters(-1);
        if(aimP.isPresent()) {
            isValid = true;
            AimingParameters aimingParameters = aimP.get();
            aimTarget = new Rotation2d(aimingParameters.getVehicleToTarget().getX(), aimingParameters.getVehicleToTarget().getY()).getDegrees();

            double unCompensatedDistance = aimingParameters.getVehicleToTarget().getTranslation().getNorm() + Constants.VisionConstants.DISTANCE_OFFSET.get();
            parameters = parametersTable.getParameters(unCompensatedDistance);
            angleTolerance = JudgeConstants.DRIVETRAIN_AIM_TOLERANCE_NEAR - JudgeConstants.DRIVETRAIN_AIM_TOLERANCE_FACTOR * unCompensatedDistance;

            if(RobotState.isTeleop()) {
                if(Util.inRange(Limelight.getInstance().getTarget().getX(), -10, 10)) {
                    swerve.getLocalizer().addMeasurement(Timer.getFPGATimestamp(), FieldConstants.hubPose.transformBy(
                                    new Transform2d(
                                            aimingParameters.getVehicleToTarget().getTranslation(),
                                            aimingParameters.getVehicleToTarget().getRotation()
                                    )
                            ),
                            new edu.wpi.first.math.geometry.Pose2d(
                                    new edu.wpi.first.math.geometry.Translation2d(0.001, 0.001),
                                    new edu.wpi.first.math.geometry.Rotation2d(0.001)));
                }
            }

            if(moveAndShoot) {
                Translation2d velocity_translational = new Translation2d(
                        aimingParameters.getVehicleVelocityToField().getX(),
                        aimingParameters.getVehicleVelocityToField().getY()
                );
                // Rotate by robot-to-goal rotation; x = radial component (positive towards goal), y = tangential component (positive means turret needs negative lead).
                velocity_translational = velocity_translational.rotateBy(GeometryAdapter.to254(aimingParameters.getVehicleToTarget()).getRotation().inverse());

                double tangential = velocity_translational.y();
                double radial = velocity_translational.x();
                double angular = aimingParameters.getVehicleVelocityToField().getRotation().getDegrees();


                double shotSpeed = unCompensatedDistance / flyTime.get() - radial;
                shotSpeed = Util.clamp(shotSpeed, 0, Double.POSITIVE_INFINITY);
                double deltaAdjustment = Units.radiansToDegrees(
                        Math.atan2(
                                -tangential, shotSpeed
                        )
                );
                aimTarget += deltaAdjustment;
                double compensatedDistance = flyTime.get() * Math.sqrt(tangential * tangential + shotSpeed * shotSpeed);
                parameters = parametersTable.getParameters(compensatedDistance);

                double drivebaseFeedforward = -(angular + Units.radiansToDegrees(tangential / unCompensatedDistance));
                swerve.setHeadingVelocityFeedforward(drivebaseFeedforward);
            }

        } else {
            isValid = false;
            AimingParameters aimingParameters = aim.getDefaultAimingParameters();
            aimTarget = new Rotation2d(aimingParameters.getVehicleToTarget().getX(), aimingParameters.getVehicleToTarget().getY()).getDegrees();
            double unCompensatedDistance = aimingParameters.getVehicleToTarget().getTranslation().getNorm() + Constants.VisionConstants.DISTANCE_OFFSET.get();
            parameters = parametersTable.getParameters(unCompensatedDistance);
        }
    }

    private void setMechanisms() {
        double rotationalVelocity = shootingController.calculate(
                swerve.getLocalizer().getLatestPose().getRotation().getDegrees(),
                aimTarget
        );
        rotationalVelocity += Math.signum(rotationalVelocity) * headingKs.get();
        if(Math.abs(shootingController.getPositionError()) > 90.0) {
            rotationalVelocity += Math.signum(rotationalVelocity) * (shootingController.getPositionError()) * headingLargeKf.get();
        }

        swerve.setOverrideRotation(rotationalVelocity);
        hood.setHoodAngle(parameters.getBackboardAngleDegree());
        shooter.setShooterRPM(parameters.getVelocityRpm());

        if (isAimed && isSpunUp && isHoodUp && isValid) {
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
        feedback.getReady().setBoolean(isAimed && isSpunUp && isHoodUp && isValid);
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

    private void updateTunables() {
        if(headingKp.hasChanged()) {
            shootingController.setP(headingKp.get());
        }
        if(headingKi.hasChanged()) {
            shootingController.setI(headingKi.get());
        }
        if(headingKd.hasChanged()) {
            shootingController.setD(headingKd.get());
        }
    }


    @Override
    public void initialize() {
        shooter.turnOff();
        trigger.lock();
        indexer.setWantFeed(false);
        swerve.clearOverrideRotation();
        shootingController.reset();
    }

    @Override
    public void execute() {
        updateShootingParameters();
        judgeStatus();
        setMechanisms();
        setIndicator();
        telemetry();
        updateTunables();
    }

    @Override
    public void end(boolean isInterrupted) {
        swerve.setLockHeading(false);
        shooter.idle();
        trigger.lock();
        indexer.setWantFeed(false);
        indexer.reset();
        hood.setHoodMinimum();
        clearTelemetry();
        swerve.setHeadingVelocityFeedforward(0.0);
        aimReady.update(false, 0.0);
        swerve.clearOverrideRotation();
        shootingController.reset();
        isLarge = false;
    }
}
