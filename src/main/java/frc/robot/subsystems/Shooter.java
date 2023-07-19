package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.team254.lib.util.Util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.utils.CTREFactory;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import lombok.Getter;
import frc.robot.Ports;

public class Shooter implements Updatable, Subsystem {
    public static class PeriodicIO {
        // INPUTS
        public double leadVelocity = 0.0;
        public double leadCurret = 0.0;
        public double leadTemperature = 0.0;
        public double followerTemperature = 0.0;

        // OUTPUTS
        public double shooterDemand = 0.0;
    }

    public PeriodicIO periodicIO = new PeriodicIO();

    @Getter
    private final TalonFX shooterLeadMotor = CTREFactory.createDefaultTalonFX(Ports.CanId.Canivore.SHOOTER_LEAD, false);
    private final TalonFX shooterFollowMotor = CTREFactory
            .createPermanentSlaveTalon(Ports.CanId.Canivore.SHOOTER_FOLLOW, Ports.CanId.Canivore.SHOOTER_LEAD, false);

    private static Shooter instance;
    private State state = State.IDLE;

    private final NetworkTableEntry shooterDemandEntry;
    private final NetworkTableEntry shooterStateEntry;
    private final NetworkTableEntry shooterVelocityEntry;
    private final NetworkTableEntry shooterCurrentEntry;
    private final NetworkTableEntry shooterLeaderTemperatureEntry;
    private final NetworkTableEntry shooterFollowerTemperatureEntry;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        shooterLeadMotor.configFactoryDefault();
        shooterLeadMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        shooterLeadMotor.configVoltageCompSaturation(12.0);
        shooterLeadMotor.enableVoltageCompensation(true);
        shooterLeadMotor.setInverted(InvertType.InvertMotorOutput);
        shooterLeadMotor.setNeutralMode(NeutralMode.Coast);
        shooterLeadMotor.config_kF(0, ShooterConstants.SHOOTER_KF.get());
        shooterLeadMotor.config_kP(0, ShooterConstants.SHOOTER_KP.get());
        shooterLeadMotor.config_kI(0, ShooterConstants.SHOOTER_KI.get());
        shooterLeadMotor.config_kD(0, ShooterConstants.SHOOTER_KD.get());
        shooterLeadMotor.config_IntegralZone(0, ShooterConstants.SHOOTER_IZONE.get());
        shooterLeadMotor.configVelocityMeasurementWindow(2);
        shooterLeadMotor.configClosedloopRamp(ShooterConstants.SHOOTER_RAMP);

        shooterFollowMotor.setInverted(InvertType.FollowMaster);
        shooterFollowMotor.setNeutralMode(NeutralMode.Coast);

        if (Constants.TUNING) {
            ShuffleboardTab dataTab = Shuffleboard.getTab("Shooter");
            shooterCurrentEntry = dataTab.add("Current", periodicIO.leadCurret).getEntry();

            shooterVelocityEntry = dataTab.add("Velocity", periodicIO.leadVelocity).getEntry();
            shooterLeaderTemperatureEntry = dataTab.add("Leader Temperature", periodicIO.leadTemperature).getEntry();
            shooterFollowerTemperatureEntry = dataTab.add("Follower Voltage", periodicIO.followerTemperature).getEntry();

            shooterDemandEntry = dataTab.add("Shooter Demand", periodicIO.shooterDemand).getEntry();
            shooterStateEntry = dataTab.add("Shooter State", state.toString()).getEntry();
        }
    }

    public void setShooterPercentage(double percentage) {
        if (state != State.OPEN_LOOP) {
            setState(State.OPEN_LOOP);
        }
        periodicIO.shooterDemand = percentage;
    }

    public void setShooterRPM(double rpm) {
        if (state != State.CLOSE_LOOP) {
            setState(State.CLOSE_LOOP);
        }
        periodicIO.shooterDemand = rpm;
    }

    public void turnOff() {
        setState(State.OFF);
    }

    public void idle() { setState(State.IDLE); }

    public double getShooterRPM() {
        return periodicIO.leadVelocity;
    }

    public synchronized boolean spunUp() {
        if (periodicIO.shooterDemand > 0) {
            return Util.epsilonEquals(
                    getShooterRPM(),
                    periodicIO.shooterDemand,
                    ShooterConstants.SHOOTER_ERROR_TOLERANCE);
        }
        return false;
    }

    @Override
    public synchronized void read(double time, double dt) {
        periodicIO.leadCurret = shooterLeadMotor.getSupplyCurrent();
        periodicIO.leadVelocity = Conversions.falconToRPM(shooterLeadMotor.getSelectedSensorVelocity(), ShooterConstants.SHOOTER_GEAR_RATIO);
        periodicIO.leadTemperature = shooterLeadMotor.getTemperature();
        periodicIO.followerTemperature = shooterFollowMotor.getTemperature();
    }

    private void updateShooterStates() {
        switch (state) {
            case CLOSE_LOOP:
                break;
            case OPEN_LOOP:
                periodicIO.shooterDemand = Math.min(periodicIO.shooterDemand, 1.0);
                break;
            case IDLE:
                periodicIO.shooterDemand = 254.0;
                break;
            case OFF:
            default:
                periodicIO.shooterDemand = 0.0;
                break;
        }
    }

    @Override
    public synchronized void update(double time, double dt) {
        updateShooterStates();

        if (ShooterConstants.SHOOTER_KP.hasChanged()) {
            shooterLeadMotor.config_kP(0, ShooterConstants.SHOOTER_KP.get());
        }
        if (ShooterConstants.SHOOTER_KI.hasChanged()) {
            shooterLeadMotor.config_kI(0, ShooterConstants.SHOOTER_KI.get());
        }
        if (ShooterConstants.SHOOTER_KD.hasChanged()) {
            shooterLeadMotor.config_kD(0, ShooterConstants.SHOOTER_KD.get());
        }
        if (ShooterConstants.SHOOTER_KF.hasChanged()) {
            shooterLeadMotor.config_kF(0, ShooterConstants.SHOOTER_KF.get());
        }

    }

    @Override
    public synchronized void write(double time, double dt) {
        if (getState() == State.OPEN_LOOP || getState() == State.OFF) {
            shooterLeadMotor.set(ControlMode.PercentOutput, periodicIO.shooterDemand);
        } else {
            System.out.println(periodicIO.shooterDemand);
            shooterLeadMotor.set(ControlMode.Velocity, Conversions.RPMToFalcon(
                    periodicIO.shooterDemand,
                    ShooterConstants.SHOOTER_GEAR_RATIO
            ));
        }
    }

    @Override
    public synchronized void telemetry() {
        if (!Constants.TUNING) return;
        shooterDemandEntry.setDouble(periodicIO.shooterDemand);
        shooterStateEntry.setString(state.toString());
        shooterVelocityEntry.setDouble(periodicIO.leadVelocity);
        shooterCurrentEntry.setDouble(periodicIO.leadCurret);
        shooterLeaderTemperatureEntry.setDouble(periodicIO.leadTemperature);
        shooterFollowerTemperatureEntry.setDouble(periodicIO.followerTemperature);
    }

    @Override
    public synchronized void start() {

    }

    @Override
    public synchronized void stop() {
        setShooterPercentage(0.0);
    }

    public enum State {
        OFF,
        IDLE,
        OPEN_LOOP,
        CLOSE_LOOP
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }
}
