package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.team254.lib.util.MovingAverage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import lombok.Getter;
import lombok.Setter;
import lombok.Synchronized;
import org.frcteam6328.utils.TunableNumber;
import org.frcteam6941.control.HolonomicDriveSignal;
import org.frcteam6941.control.HolonomicTrajectoryFollower;
import org.frcteam6941.drivers.DummyGyro;
import org.frcteam6941.drivers.Gyro;
import org.frcteam6941.drivers.Pigeon2Gyro;
import org.frcteam6941.localization.Localizer;
import org.frcteam6941.localization.SwerveDeltaCoarseLocalizer;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.swerve.*;
import org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.frcteam6941.utils.AngleNormalization;

import java.util.Optional;

/**
 * Rectangular Swerve Drivetrain composed of SJTU Swerve Module MK5s.
 */
public class Swerve implements Updatable, Subsystem {
    private final SwerveModuleBase[] swerveMods;
    private final SwerveDriveKinematics swerveKinematics;
    private final SwerveDeltaCoarseLocalizer swerveLocalizer;

    @Getter
    private final Gyro gyro;

    // Snap Rotation Controller
    private final ProfiledPIDController headingController;
    private boolean isLockHeading;
    private double headingTarget = 0.0;
    @Getter @Setter
    private Double overrideRotation = null;
    @Getter @Setter
    private double headingVelocityFeedforward = 0.00;

    // Path Following Controller
    @Getter
    private final HolonomicTrajectoryFollower trajectoryFollower;

    // Control Targets
    private HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(new Translation2d(), 0.0, true, false);
    private SwerveSetpoint setpoint;
    private SwerveSetpoint previousSetpoint;
    @Getter
    private KinematicLimits kinematicLimits;
    private final SwerveSetpointGenerator generator;

    // System Status
    private final MovingAverage pitchVelocity;
    private final MovingAverage rollVelocity;
    private final MovingAverage yawVelocity;

    private final double headingKp = 0.015000;
    private final double headingKi = 0.000070;
    private final double headingKd = 0.002000;


    // Logging
    private final NetworkTable dataTable = NetworkTableInstance.getDefault().getTable("Swerve");

    private State state = State.DRIVE;

    private static Swerve instance;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        if (RobotBase.isReal()) {
            swerveMods = new SwerveModuleBase[]{
                    new SJTUMK5iModule(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD0),
                    new SJTUMK5iModule(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD1),
                    new SJTUMK5iModule(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD2),
                    new SJTUMK5iModule(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD3)
            };
            gyro = new Pigeon2Gyro(0);
        } else {
            swerveMods = new SwerveModuleBase[]{
                    new SJTUMK5iModuleSim(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD0),
                    new SJTUMK5iModuleSim(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD1),
                    new SJTUMK5iModuleSim(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD2),
                    new SJTUMK5iModuleSim(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD3)
            };
            gyro = new DummyGyro(Constants.LOOPER_DT);
        }

        swerveKinematics = Constants.SwerveConstants.DRIVETRAIN_CONSTANTS.getDrivetrainKinematics();
        swerveLocalizer = new SwerveDeltaCoarseLocalizer(swerveKinematics, 50, 20, 20);

        gyro.setYaw(0.0);
        swerveLocalizer.reset(new Pose2d());

        yawVelocity = new MovingAverage(10);
        pitchVelocity = new MovingAverage(10);
        rollVelocity = new MovingAverage(10);

        setpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());
        previousSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());
        generator = new SwerveSetpointGenerator(
                Constants.SwerveConstants.DRIVETRAIN_CONSTANTS.getDrivetrainModPositions());
        kinematicLimits = SwerveConstants.DRIVETRAIN_UNCAPPED;

        headingController = new ProfiledPIDController(
                headingKp, headingKi, headingKd,
                new TrapezoidProfile.Constraints(600, 720));
        headingController.setIntegratorRange(-0.5, 0.5);
        headingController.enableContinuousInput(0, 360.0);

        trajectoryFollower = new HolonomicTrajectoryFollower(
                new PIDController(1.2, 0.0, 0.0),
                new PIDController(1.2, 0.0, 0.0),
                headingController,
                Constants.SwerveConstants.DRIVETRAIN_FEEDFORWARD);
    }

    /**
     * Core methods to update the odometry of swerve based on module states.
     *
     * @param time Current time stamp.
     * @param dt   Delta time between updates.
     */
    private void updateOdometry(double time, double dt) {
        swerveLocalizer.updateWithTime(time, dt, gyro.getYaw(), getModuleStates());
    }

    /**
     * Core method to update swerve modules according to the
     * {@link HolonomicDriveSignal} given.
     *
     * @param driveSignal The holonomic drive signal.
     * @param dt          Delta time between updates.
     */
    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        ChassisSpeeds desiredChassisSpeed;

        if (driveSignal == null) {
            desiredChassisSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
            driveSignal = new HolonomicDriveSignal(new Translation2d(), 0.0, true, false);
        } else {
            double x = driveSignal.getTranslation().getX();
            double y = driveSignal.getTranslation().getY();
            double rotation = driveSignal.getRotation();
            Rotation2d robotAngle = swerveLocalizer.getLatestPose().getRotation();

            if (driveSignal.isFieldOriented())
                desiredChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, robotAngle);
            else desiredChassisSpeed = new ChassisSpeeds(x, y, rotation);

            if (!driveSignal.isOpenLoop()) {
                desiredChassisSpeed.vxMetersPerSecond = desiredChassisSpeed.vxMetersPerSecond
                        * kinematicLimits.kMaxDriveVelocity;
                desiredChassisSpeed.vyMetersPerSecond = desiredChassisSpeed.vyMetersPerSecond
                        * kinematicLimits.kMaxDriveVelocity;
                desiredChassisSpeed.omegaRadiansPerSecond = desiredChassisSpeed.omegaRadiansPerSecond
                        * Units.degreesToRadians(SwerveConstants.DRIVETRAIN_MAX_ROTATION_VELOCITY);
            }
        }

        Twist2d twist = new Pose2d().log(new Pose2d(
                        new Translation2d(
                                desiredChassisSpeed.vxMetersPerSecond * Constants.LOOPER_DT,
                                desiredChassisSpeed.vyMetersPerSecond * Constants.LOOPER_DT
                        ),
                        new Rotation2d(
                                desiredChassisSpeed.omegaRadiansPerSecond * Constants.LOOPER_DT)
                )
        );

        desiredChassisSpeed = new ChassisSpeeds(
                twist.dx / Constants.LOOPER_DT,
                twist.dy / Constants.LOOPER_DT,
                twist.dtheta / Constants.LOOPER_DT);

        setpoint = generator.generateSetpoint(
                kinematicLimits, previousSetpoint, desiredChassisSpeed, dt);
        previousSetpoint = setpoint;
        for (SwerveModuleBase mod : swerveMods) {
            mod.setDesiredState(setpoint.mModuleStates[mod.getModuleNumber()], driveSignal.isOpenLoop(), false);
        }
    }

    @Synchronized
    public double getYawVelocity() {
        return yawVelocity.getAverage();
    }

    @Synchronized
    public double getPitchVelocity() {
        return pitchVelocity.getAverage();
    }

    @Synchronized
    public double getRollVelocity() {
        return rollVelocity.getAverage();
    }

    /**
     * Core method to drive the swerve drive. Note that any trajectory following
     * signal will be canceled when this method is called.
     *
     * @param translationalVelocity Normalized translation vector of the swerve
     *                              drive.
     * @param rotationalVelocity    Normalized rotational magnitude of the swerve
     *                              drive.
     * @param isFieldOriented       Is the drive signal field oriented.
     */
    public void drive(Translation2d translationalVelocity, double rotationalVelocity,
                      boolean isFieldOriented, boolean isOpenLoop) {
        driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented, isOpenLoop);
    }

    public void brake() {
        setState(State.BRAKE);
    }

    public void normal() {
        setState(State.DRIVE);
    }

    public void empty() {
        setState(State.EMPTY);
    }

    public void follow(PathPlannerTrajectory trajectory, boolean lockAngle, boolean requiredOnTarget) {
        resetHeadingController();
        trajectoryFollower.setLockAngle(lockAngle);
        trajectoryFollower.setRequiredOnTarget(requiredOnTarget);
        if (trajectory == null) {
            this.setState(State.PATH_FOLLOWING);
            trajectoryFollower.cancel();
        }
        trajectoryFollower.follow(trajectory);
    }

    public void stopMovement() {
        driveSignal = new HolonomicDriveSignal(new Translation2d(), 0.0, true, false);
    }

    public void setKinematicsLimit(KinematicLimits limit) {
        kinematicLimits = limit;
    }

    public void resetPose(Pose2d resetPose) {
        gyro.setYaw(resetPose.getRotation().getDegrees());
        swerveLocalizer.reset(resetPose);
    }

    /**
     * Set the state of the module independently.
     *
     * @param desiredStates The states of the model.
     * @param isOpenLoop    Whether to use open loop control
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean overrideMotion) {
        if (isOpenLoop) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1.0);
        }

        for (SwerveModuleBase mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], isOpenLoop, overrideMotion);
        }
    }

    /**
     * Convenience method to set the wheels in X shape to resist impacts.
     */
    private void setModuleStatesBrake() {
        for (SwerveModuleBase mod : swerveMods) {
            Translation2d modulePosition = SwerveConstants.DRIVETRAIN_CONSTANTS.getDrivetrainModPositions()[mod.getModuleNumber()];
            Rotation2d angle = new Rotation2d(modulePosition.getX(), modulePosition.getY());
            mod.setDesiredState(new SwerveModuleState(0.0, angle.plus(Rotation2d.fromDegrees(180.0))), false, true);
        }
    }

    public void resetYaw(double degree) {
        this.gyro.setYaw(degree);
    }

    public void resetRoll(double degree) {
        this.gyro.setRoll(degree);
    }

    public void resetPitch(double degree) {
        this.gyro.setPitch(degree);
    }

    public Localizer getLocalizer() {
        return this.swerveLocalizer;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveMods.length];
        for (SwerveModuleBase mod : swerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModuleBase[] getSwerveMods() {
        return swerveMods;
    }

    /*
     * Reset heading controller according to current drivetrain status.
     */
    public void resetHeadingController() {
        headingController.reset(
                swerveLocalizer.getLatestPose().getRotation().getDegrees(),
                getYawVelocity());
    }

    /**
     * Return if the swerve drive has a set heading target.
     *
     * @return If swerve is in lock heading.
     */
    public boolean isLockHeading() {
        return this.isLockHeading;
    }

    /**
     * Set if swerve will enter lock heading.
     *
     * @param status Boolean value for enabling or disabling lock heading.
     */
    public void setLockHeading(boolean status) {
        if (this.isLockHeading != status) {
            headingController.reset(gyro.getYaw().getDegrees(), getYawVelocity());
        }
        this.isLockHeading = status;
        this.headingVelocityFeedforward = 0.0;
    }

    public synchronized void setHeadingTarget(double heading) {
        double target = heading;
        double position = gyro.getYaw().getDegrees();

        while (position - target > 180) {
            target += 360;
        }

        while (target - position > 180) {
            target -= 360;
        }

        headingTarget = target;
    }

    /**
     * Get the lock heading target for the swerve drive.
     *
     * @return The desired heading target from 0 to 360 in degrees.
     */
    public double getHeadingTarget() {
        return this.headingTarget;
    }


    public boolean isHeadingOnTarget() {
        return this.headingController.atSetpoint();
    }

    public void clearOverrideRotation() {
        overrideRotation = null;
    }

    @Override
    public void read(double time, double dt) {
        updateOdometry(time, dt);
    }

    @Override
    public void update(double time, double dt) {
        Optional<HolonomicDriveSignal> trajectorySignal = trajectoryFollower.update(
                swerveLocalizer.getLatestPose(),
                swerveLocalizer.getMeasuredVelocity().getTranslation(),
                swerveLocalizer.getMeasuredVelocity().getRotation().getDegrees(),
                time, dt);
        if (trajectorySignal.isPresent()) {
            setState(State.PATH_FOLLOWING);
            driveSignal = trajectorySignal.get();
        } else if (isLockHeading) {
            headingTarget = AngleNormalization.placeInAppropriate0To360Scope(gyro.getYaw().getDegrees(), headingTarget);
            double rotation = headingController.calculate(gyro.getYaw().getDegrees(), new TrapezoidProfile.State(
                    headingTarget, headingVelocityFeedforward
            ));
            driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation(), rotation,
                    driveSignal.isFieldOriented(), driveSignal.isOpenLoop());
        } else if (overrideRotation != null) {
            driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation(), overrideRotation,
                    driveSignal.isFieldOriented(), driveSignal.isOpenLoop());
        }

        rollVelocity.addNumber(gyro.getRaw()[0]);
        pitchVelocity.addNumber(gyro.getRaw()[1]);
        yawVelocity.addNumber(gyro.getRaw()[2]);
    }

    @Override
    public void write(double time, double dt) {
        switch (state) {
            case BRAKE:
                setModuleStatesBrake();
                break;
            case DRIVE:
            case PATH_FOLLOWING:
                updateModules(driveSignal, dt);
                break;
            case EMPTY:
                break;
        }
    }

    @Override
    public void telemetry() {
        Pose2d latestPose = swerveLocalizer.getLatestPose();
        dataTable.getEntry("Pose").setDoubleArray(
                new double[]{
                        latestPose.getX(), latestPose.getY(), latestPose.getRotation().getDegrees()
                });

        if (Constants.TUNING) {
            for (SwerveModuleBase module : swerveMods) {
                SmartDashboard.putNumber("Mod" + module.getModuleNumber(), module.getState().angle.getDegrees());
                SmartDashboard.putNumber("Speed Mod" + module.getModuleNumber(), module.getState().speedMetersPerSecond);
                SmartDashboard.putNumber("Mod S" + module.getModuleNumber(), Math.abs(module.getState().speedMetersPerSecond));
                SmartDashboard.putNumber("Mod SD" + module.getModuleNumber(),
                        Math.abs(setpoint.mModuleStates[module.getModuleNumber()].speedMetersPerSecond));
            }

            Pose2d velocity = swerveLocalizer.getSmoothedVelocity();
            SmartDashboard.putNumberArray("Velocity", new double[]{
                    velocity.getX(), velocity.getY(), velocity.getRotation().getDegrees()
            });

            double tempSum = 0;
            for(SwerveModuleBase mod: getSwerveMods()) {
                tempSum += Math.abs(mod.getTick());
            }
            SmartDashboard.putNumber("Ticks", tempSum /= getSwerveMods().length);
        }
    }

    @Override
    public void stop() {
        stopMovement();
        setState(State.DRIVE);
    }

    @Override
    public void simulate(double time, double dt) {
        read(time, dt);
        gyro.setYaw(gyro.getYaw().rotateBy(new Rotation2d(dt * setpoint.mChassisSpeeds.omegaRadiansPerSecond))
                .getDegrees());
    }

    public enum State {
        BRAKE, DRIVE, PATH_FOLLOWING, EMPTY
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }
}
