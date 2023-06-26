package frc.robot.subsystems;

import java.util.Optional;

import org.frcteam6941.control.HolonomicDriveSignal;
import org.frcteam6941.control.HolonomicTrajectoryFollower;
import org.frcteam6941.drivers.Gyro;
import org.frcteam6941.drivers.Pigeon2Gyro;
import org.frcteam6941.localization.Localizer;
import org.frcteam6941.localization.SwerveLocalizer;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUMK5iModule;
import org.frcteam6941.swerve.SJTUMK5iModuleSim;
import org.frcteam6941.swerve.SwerveModuleBase;
import org.frcteam6941.swerve.SwerveSetpoint;
import org.frcteam6941.swerve.SwerveSetpointGenerator;
import org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

/**
 * Rectangular Swerve Drivetrain composed of SJTU Swerve Module MK5s. This is a
 * basic implementation of {@link SwerveDrivetrainBase}.
 */
public class Swerve extends SubsystemBase implements Updatable {
    private final SwerveModuleBase[] mSwerveMods;
    private final SwerveDriveKinematics swerveKinematics;
    private final SwerveLocalizer swerveLocalizer;

    // Path Following Controller
    private final ProfiledPIDController headingController = new ProfiledPIDController(
        0.05, 0.0, 0.0,
        new TrapezoidProfile.Constraints(360.0, 720.0)
    );
    private final HolonomicTrajectoryFollower trajectoryFollower = new HolonomicTrajectoryFollower(
        new PIDController(2.0, 0.0, 0.0),
        new PIDController(2.0, 0.0, 0.0),
        headingController,
        Constants.SwerveConstants.DRIVETRAIN_FEEDFORWARD
    );
    private final Gyro gyro;
    private static Swerve instance;

    // Control Targets
    private HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(new Translation2d(), 0.0, true, false);
    private SwerveSetpoint setpoint;
    private SwerveSetpoint previousSetpoint;
    private KinematicLimits kinematicLimits;
    private SwerveSetpointGenerator generator;

    // System Status
    private MovingAverage pitchVelocity;
    private MovingAverage rollVelocity;
    private MovingAverage yawVelocity;

    private STATE state = STATE.DRIVE;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        gyro = new Pigeon2Gyro(0);

        // Swerve hardware configurations
        if(RobotBase.isReal()) {
            mSwerveMods = new SwerveModuleBase[] {
                new SJTUMK5iModule(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD0),
                new SJTUMK5iModule(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD1),
                new SJTUMK5iModule(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD2),
                new SJTUMK5iModule(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD3)
            };
        } else {
            mSwerveMods = new SwerveModuleBase[] {
                new SJTUMK5iModuleSim(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD0),
                new SJTUMK5iModuleSim(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD1),
                new SJTUMK5iModuleSim(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD2),
                new SJTUMK5iModuleSim(SwerveConstants.DRIVETRAIN_CONSTANTS, SwerveConstants.MOD3)
            };
        }

        swerveKinematics = Constants.SwerveConstants.DRIVETRAIN_CONSTANTS.getDrivetrainKinematics();
        swerveLocalizer = new SwerveLocalizer(swerveKinematics, 100, 15, 15);

        gyro.setYaw(0.0);
        swerveLocalizer.reset(new Pose2d());

        yawVelocity = new MovingAverage(10);
        pitchVelocity = new MovingAverage(10);
        rollVelocity = new MovingAverage(10);

        headingController.enableContinuousInput(0, 360.0);
        headingController.setTolerance(1.0);

        setpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());
        previousSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());
        generator = new SwerveSetpointGenerator(Constants.SwerveConstants.DRIVETRAIN_CONSTANTS.getDrivetrainModPositions());
        kinematicLimits = SwerveConstants.DRIVETRAIN_UNCAPPED;
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

            if (driveSignal.isFieldOriented()) {
                desiredChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, robotAngle);
            } else {
                desiredChassisSpeed = new ChassisSpeeds(x, y, rotation);
            }

            if (!driveSignal.isOpenLoop()) {
                desiredChassisSpeed.vxMetersPerSecond = desiredChassisSpeed.vxMetersPerSecond * kinematicLimits.kMaxDriveVelocity;
                desiredChassisSpeed.vyMetersPerSecond = desiredChassisSpeed.vyMetersPerSecond * kinematicLimits.kMaxDriveVelocity;
            }
        }

        Twist2d twist = new Pose2d().log(new Pose2d(
                new Translation2d(desiredChassisSpeed.vxMetersPerSecond * Constants.LOOPER_DT,
                        desiredChassisSpeed.vyMetersPerSecond * Constants.LOOPER_DT),
                new Rotation2d(desiredChassisSpeed.omegaRadiansPerSecond * Constants.LOOPER_DT)));

        desiredChassisSpeed = new ChassisSpeeds(
            twist.dx / Constants.LOOPER_DT,
            twist.dy / Constants.LOOPER_DT,
            twist.dtheta / Constants.LOOPER_DT
        );

        setpoint = generator.generateSetpoint(
            kinematicLimits, previousSetpoint, desiredChassisSpeed, dt
        );
        previousSetpoint = setpoint;

        if (driveSignal.isOpenLoop()) {
            for (SwerveModuleBase mod : mSwerveMods) {
                SwerveModuleState temp = setpoint.mModuleStates[mod.getModuleNumber()];
                temp.speedMetersPerSecond = temp.speedMetersPerSecond / kinematicLimits.kMaxDriveVelocity;
                mod.setDesiredState(setpoint.mModuleStates[mod.getModuleNumber()], true, false);
            }
        } else {
            for (SwerveModuleBase mod : mSwerveMods) {
                mod.setDesiredState(setpoint.mModuleStates[mod.getModuleNumber()], false, false);
            }
        }
        
    }

    public synchronized double getYawVelocity() {
        return yawVelocity.getAverage();
    }

    public synchronized double getPitchVelocity() {
        return pitchVelocity.getAverage();
    }

    public synchronized double getRollVelocity() {
        return rollVelocity.getAverage();
    }

    /**
     * Core method to drive the swerve drive. Note that any trajectory following
     * signal will be canceled when this method is called.
     * 
     * @param translationalVelocity Translation vector of the swerve drive.
     * @param rotationalVelocity    Rotational magnitude of the swerve drive.
     * @param isFieldOriented       Is the drive signal field oriented.
     */
    public void drive(Translation2d translationalVelocity, double rotationalVelocity,
            boolean isFieldOriented, boolean isOpenLoop) {
        driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity,
                isFieldOriented, isOpenLoop);
    }

    public void brake() {
        setState(STATE.BRAKE);
    }

    public void normal() {
        setState(STATE.DRIVE);
    }

    public void empty() {
        setState(STATE.EMPTY);
    }

    public void follow(PathPlannerTrajectory trajectory) {
        if(trajectory != null) {
            resetHeadingController();
            this.setState(STATE.PATH_FOLLOWING);
            trajectoryFollower.cancel();
            trajectoryFollower.follow(trajectory);
        }
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
     * @param isOpenLoop    If use open loop control
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean overrideMotion) {
        if(isOpenLoop) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1.0);
        }
        
        for (SwerveModuleBase mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], isOpenLoop, overrideMotion);
        }
    }

    /**
     * Convenience method to set the wheels in X shape to resist impacts.
     */
    private void setModuleStatesBrake() {
        for (SwerveModuleBase mod : mSwerveMods) {
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
        SwerveModuleState[] states = new SwerveModuleState[mSwerveMods.length];
        for (SwerveModuleBase mod : mSwerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public synchronized void resetHeadingController() {
        headingController.reset(
            swerveLocalizer.getLatestPose().getRotation().getDegrees(),
            getYawVelocity()
        );
    }

    @Override
    public synchronized void read(double time, double dt) {
        updateOdometry(time, dt);
    }

    @Override
    public synchronized void update(double time, double dt) {
        Optional<HolonomicDriveSignal> trajectorySignal = trajectoryFollower.update(
            swerveLocalizer.getLatestPose(),
            swerveLocalizer.getMeasuredVelocity().getTranslation(),
            swerveLocalizer.getMeasuredVelocity().getRotation().getDegrees(),
            time, dt
        );
        if (trajectorySignal.isPresent()) {
            setState(STATE.PATH_FOLLOWING);
            driveSignal = trajectorySignal.get();
        }

        gyro.updateIO();
        rollVelocity.addNumber(gyro.getRaw()[0]);
        pitchVelocity.addNumber(gyro.getRaw()[1]);
        yawVelocity.addNumber(gyro.getRaw()[2]);
    }

    @Override
    public synchronized void write(double time, double dt) {
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
    public synchronized void telemetry() {
        Logger.getInstance().processInputs("Drivetrain/Gyro", gyro.getIO());
    }

    @Override
    public synchronized void start() {

    }

    @Override
    public synchronized void stop() {
        stopMovement();
        setState(STATE.DRIVE);
    }

    @Override
    public synchronized void simulate(double time, double dt) {
        read(time, dt);
    }

    public enum STATE {
        BRAKE, DRIVE, PATH_FOLLOWING, EMPTY
    };

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
