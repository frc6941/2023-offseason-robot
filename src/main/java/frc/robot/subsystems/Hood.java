package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.util.Util;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.utils.CTREFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HoodConstants;
import frc.robot.Ports;

public class Hood implements Updatable, Subsystem {
    public static class PeriodicIO {
        // INPUT
        public double hoodCurrent;
        public double hoodPosition;
        public double hoodVelocity;
        public double hoodVoltage;

        // OUTPUT
        public double hoodDemand;
    }

    public PeriodicIO periodicIO = new PeriodicIO();

    public TalonFX hoodMotor = CTREFactory.createDefaultTalonFX(Ports.CanId.Canivore.HOOD, false);

    private boolean isCalibrated = false;

    private Hood() {
        hoodMotor.configFactoryDefault();
        hoodMotor.setNeutralMode(NeutralMode.Coast);
        hoodMotor.config_kP(0, HoodConstants.HOOD_KP);
        hoodMotor.config_kI(0, HoodConstants.HOOD_KI);
        hoodMotor.config_kD(0, HoodConstants.HOOD_KD);
        hoodMotor.config_kF(0, HoodConstants.HOOD_KF);
        hoodMotor.configMotionCruiseVelocity(HoodConstants.HOOD_CRUISE_V);
        hoodMotor.configMotionAcceleration(HoodConstants.HOOD_CRUISE_ACC);
        hoodMotor.enableVoltageCompensation(true);
        hoodMotor.config_IntegralZone(0, 50);
        hoodMotor.configNeutralDeadband(0.01);
        hoodMotor.configMotionSCurveStrength(HoodConstants.HOOD_S_STRENGTH);
        hoodMotor.setInverted(true);
    }

    private STATE state = STATE.HOMING;

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    private static Hood instance;

    public synchronized void resetHood(double angle) {
        hoodMotor.setSelectedSensorPosition(
                Conversions.degreesToFalcon(angle, HoodConstants.HOOD_GEAR_RATIO));
        isCalibrated = true;
    }

    public synchronized void setHoodPercentage(double power) {
        if (getState() != STATE.PERCENTAGE) {
            setState(STATE.PERCENTAGE);
        }
        periodicIO.hoodDemand = power;
    }

    public synchronized void setHoodAngle(double angle) {
        if (getState() != STATE.ANGLE) {
            setState(STATE.ANGLE);
        }
        angle = Util.clamp(angle, HoodConstants.HOOD_MINIMUM_ANGLE, HoodConstants.HOOD_MAXIMUM_ANGLE);
        periodicIO.hoodDemand = angle;
    }

    public synchronized double getHoodAngle() {
        return Conversions.falconToDegrees(periodicIO.hoodPosition, HoodConstants.HOOD_GEAR_RATIO);
    }

    public synchronized boolean isCalibrated() {
        return isCalibrated;
    }

    private void updateHoodStates() {
        if (!isCalibrated) {
            setState(STATE.HOMING);
        }

        switch (state) {
            case HOMING:
                periodicIO.hoodDemand = -0.2;
                if (isCalibrated) {
                    setState(STATE.OFF);
                }
                if (periodicIO.hoodCurrent > HoodConstants.HOOD_HOMING_CURRENT_THRESHOLD) {
                    resetHood(HoodConstants.HOOD_MINIMUM_ANGLE - 0.5);
                }
                break;
            case PERCENTAGE:
                periodicIO.hoodDemand = Util.limit(periodicIO.hoodDemand, -1.0, 1.0);
                break;
            case ANGLE:
                periodicIO.hoodDemand = Util.limit(periodicIO.hoodDemand, HoodConstants.HOOD_MINIMUM_ANGLE, HoodConstants.HOOD_MAXIMUM_ANGLE);
                break;
            case OFF:
                periodicIO.hoodDemand = 0.0;
                break;
        }
    }

    @Override
    public synchronized void read(double time, double dt) {
        periodicIO.hoodCurrent = hoodMotor.getStatorCurrent();
        periodicIO.hoodPosition = hoodMotor.getSelectedSensorPosition();
        periodicIO.hoodVelocity = hoodMotor.getSelectedSensorVelocity();
        periodicIO.hoodVoltage = hoodMotor.getMotorOutputVoltage();
    }

    @Override
    public synchronized void update(double time, double dt) {
        updateHoodStates();
    }

    @Override
    public synchronized void write(double time, double dt) {
        switch (state) {
            case HOMING:
            case PERCENTAGE:
                hoodMotor.set(ControlMode.PercentOutput, periodicIO.hoodDemand);
                break;
            case ANGLE:
                hoodMotor.set(ControlMode.MotionMagic,
                        Conversions.degreesToFalcon(periodicIO.hoodDemand, HoodConstants.HOOD_GEAR_RATIO));
                break;
            case OFF:
                hoodMotor.set(ControlMode.PercentOutput, 0.0);
                break;
        }
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putNumber("Hood Demand", periodicIO.hoodDemand);
        SmartDashboard.putNumber("Hood Angle", getHoodAngle());
    }

    @Override
    public synchronized void start() {
        isCalibrated = false;
        setState(STATE.HOMING);
    }

    @Override
    public synchronized void stop() {
    }


    public enum STATE {
        OFF,
        HOMING,
        PERCENTAGE,
        ANGLE
    }

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}

