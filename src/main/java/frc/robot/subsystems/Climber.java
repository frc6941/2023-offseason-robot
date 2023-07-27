// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Ports;
import lombok.Getter;
import lombok.Setter;
import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.utils.CTREFactory;

/**
 * Add your docs here.
 */
public class Climber implements Updatable, Subsystem {
    public static class PeriodicIO {
        // INPUT
        public double hookVoltage;
        public double hookCurrent;
        public double hookPosition;
        public double hookVelocity;

        public double pusherVoltage;
        public double pusherCurrent;
        public double pusherPosition;
        public double pusherVelocity;

        // OUTPUT
        public double hookDemand;

        public double pusherDemand;
    }

    public PeriodicIO periodicIO = new PeriodicIO();

    public TalonFX hookMotor = CTREFactory.createDefaultTalonFX(Ports.CanId.Canivore.CLIMBER_HOOK, false);
    public TalonFX pusherMotor = CTREFactory.createDefaultTalonFX(Ports.CanId.Canivore.CLIMBER_PUSHER, false);

    // private boolean isCalibrated = false;
    @Setter
    @Getter
    private HookState hookState = HookState.HOOK_OFF;
    @Setter
    @Getter
    private PusherState pusherState = PusherState.PUSHER_OFF;

    public enum HookState {
        HOOK_ANGLE,
        HOOK_PERCENTAGE,
        HOOK_OFF
    }

    public enum PusherState {
        PUSHER_ANGLE,
        PUSHER_PERCENTAGE,
        PUSHER_OFF
    }

    private static Climber instance;

    private Climber() {
        hookMotor.configFactoryDefault();
        hookMotor.setNeutralMode(NeutralMode.Brake);
        hookMotor.config_kP(0, ClimberConstants.HOOK_KP.get());
        hookMotor.config_kI(0, ClimberConstants.HOOK_KI.get());
        hookMotor.config_kD(0, ClimberConstants.HOOK_KD.get());
        hookMotor.config_kF(0, ClimberConstants.HOOK_KF.get());
        hookMotor.configMotionCruiseVelocity(ClimberConstants.HOOK_CRUISE_V);
        hookMotor.configMotionAcceleration(ClimberConstants.HOOK_CRUISE_ACC);
        hookMotor.enableVoltageCompensation(true);
        hookMotor.config_IntegralZone(0, 50);
        hookMotor.configNeutralDeadband(0.01);
        hookMotor.configMotionSCurveStrength(ClimberConstants.HOOK_S_STRENGTH);

        pusherMotor.configFactoryDefault();
        pusherMotor.setNeutralMode(NeutralMode.Coast);
        pusherMotor.config_kP(0, ClimberConstants.PUSHER_KP.get());
        pusherMotor.config_kI(0, ClimberConstants.PUSHER_KI.get());
        pusherMotor.config_kD(0, ClimberConstants.PUSHER_KD.get());
        pusherMotor.config_kF(0, ClimberConstants.PUSHER_KF.get());
        pusherMotor.configMotionCruiseVelocity(ClimberConstants.PUSHER_CRUISE_V);
        pusherMotor.configMotionAcceleration(ClimberConstants.PUSHER_CRUISE_ACC);
        pusherMotor.enableVoltageCompensation(true);
        pusherMotor.config_IntegralZone(0, 50);
        pusherMotor.configNeutralDeadband(0.01);
        pusherMotor.configMotionSCurveStrength(ClimberConstants.PUSHER_S_STRENGTH);

    }

    public synchronized void resetHook(double angle) {
        hookMotor.setSelectedSensorPosition(
                Conversions.degreesToFalcon(angle, ClimberConstants.HOOK_GEAR_RATIO));
    }

    public synchronized void resetPusher(double angle) {
        pusherMotor.setSelectedSensorPosition(
                Conversions.degreesToFalcon(angle, ClimberConstants.PUSHER_GEAR_RATIO));
    }

    public synchronized void setHookAngle(double angle) {
        if (getHookState() != HookState.HOOK_ANGLE) {
            setHookState(HookState.HOOK_ANGLE);
        }
        angle = Util.clamp(
                angle, ClimberConstants.HOOK_MIN_ANGLE, ClimberConstants.HOOK_MAX_ANGLE);
        periodicIO.hookDemand = angle;
    }

    public synchronized void setPusherAngle(double angle) {
        if (getPusherState() != PusherState.PUSHER_ANGLE) {
            setPusherState(PusherState.PUSHER_ANGLE);
        }
        angle = Util.clamp(
                angle, ClimberConstants.PUSHER_MIN_ANGLE, ClimberConstants.PUSHER_MAX_ANGLE);
        periodicIO.pusherDemand = angle;
    }

    public synchronized void jogHook(double delta) {
        setHookAngle(getHookAngle() + delta);
    }

    public synchronized void jogPusher(double delta) {
        setPusherAngle(getPusherAngle() + delta);
    }

    public synchronized void setHookPercentage(double power) {
        if (getHookState() != HookState.HOOK_PERCENTAGE) {
            setHookState(HookState.HOOK_PERCENTAGE);
        }
        periodicIO.hookDemand = power;
    }

    public synchronized void setPusherPercentage(double power) {
        if (getPusherState() != PusherState.PUSHER_PERCENTAGE) {
            setPusherState(PusherState.PUSHER_PERCENTAGE);
        }
        periodicIO.pusherDemand = power;
    }

    public synchronized void lockHook() {
        setHookAngle(getHookAngle());
    }

    public synchronized void lockPusher() {
        setPusherAngle(getPusherAngle());
    }

    public synchronized void brakeClimber() {
        pusherMotor.setNeutralMode(NeutralMode.Brake);
        hookMotor.setNeutralMode(NeutralMode.Brake);
    }

    public synchronized void coastClimber() {
        pusherMotor.setNeutralMode(NeutralMode.Coast);
        hookMotor.setNeutralMode(NeutralMode.Coast);
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public synchronized void setHookMinimum() {
        setHookAngle(ClimberConstants.HOOK_MIN_ANGLE);
    }

    public synchronized void setPusherMinimum() {
        setPusherAngle(ClimberConstants.PUSHER_MIN_ANGLE);
    }

    public synchronized double getHookAngle() {
        return Conversions.falconToDegrees(periodicIO.hookPosition, ClimberConstants.HOOK_GEAR_RATIO);
    }

    public synchronized double getPusherAngle() {
        return Conversions.falconToDegrees(periodicIO.pusherPosition, ClimberConstants.PUSHER_GEAR_RATIO);
    }

    private void updateHookStates() {
        switch (hookState) {
            case HOOK_PERCENTAGE:
                periodicIO.hookDemand = Util.clamp(periodicIO.hookDemand, -1.0, 1.0);
                break;
            case HOOK_ANGLE:
                periodicIO.hookDemand = Util.clamp(periodicIO.hookDemand, ClimberConstants.HOOK_MIN_ANGLE,
                        ClimberConstants.HOOK_MAX_ANGLE);
                break;
            case HOOK_OFF:
                periodicIO.hookDemand = 0.0;
                break;
        }
    }

    private void updatePusherStates() {
        switch (pusherState) {
            case PUSHER_PERCENTAGE:
                periodicIO.pusherDemand = Util.clamp(periodicIO.pusherDemand, -1.0, 1.0);
                break;
            case PUSHER_ANGLE:
                periodicIO.pusherDemand = Util.clamp(periodicIO.pusherDemand, ClimberConstants.PUSHER_MIN_ANGLE,
                        ClimberConstants.PUSHER_MAX_ANGLE);
                break;
            case PUSHER_OFF:
                periodicIO.hookDemand = 0.0;
                break;
        }
    }

    @Override
    public synchronized void read(double time, double dt) {
        periodicIO.hookCurrent = hookMotor.getStatorCurrent();
        periodicIO.hookPosition = hookMotor.getSelectedSensorPosition();
        periodicIO.hookVelocity = hookMotor.getSelectedSensorVelocity();
        periodicIO.pusherCurrent = pusherMotor.getStatorCurrent();
        periodicIO.pusherPosition = pusherMotor.getSelectedSensorPosition();
        periodicIO.pusherVelocity = pusherMotor.getSelectedSensorVelocity();
    }

    @Override
    public synchronized void update(double time, double dt) {
        updateHookStates();
        updatePusherStates();
    }

    @Override
    public synchronized void write(double time, double dt) {
        switch (hookState) {
            case HOOK_PERCENTAGE:
                hookMotor.set(ControlMode.PercentOutput, periodicIO.hookDemand);
                break;
            case HOOK_ANGLE:
                hookMotor.set(ControlMode.Position, Conversions.degreesToFalcon(
                        periodicIO.hookDemand,
                        ClimberConstants.HOOK_GEAR_RATIO
                ));
                break;
            case HOOK_OFF:
            default:
                hookMotor.set(ControlMode.PercentOutput, 0.0);
                break;
        }

        switch (pusherState) {
            case PUSHER_PERCENTAGE:
                pusherMotor.set(ControlMode.PercentOutput, periodicIO.pusherDemand);
                break;
            case PUSHER_ANGLE:
                pusherMotor.set(ControlMode.Position, Conversions.degreesToFalcon(
                        periodicIO.pusherDemand,
                        ClimberConstants.PUSHER_GEAR_RATIO
                ));
                break;
            case PUSHER_OFF:
            default:
                pusherMotor.set(ControlMode.PercentOutput, 0.0);
                break;
        }
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putNumber("Hook Demand", periodicIO.hookDemand);
        SmartDashboard.putNumber("Hook Angle", getHookAngle());
        SmartDashboard.putNumber("Pusher Demand", periodicIO.pusherDemand);
        SmartDashboard.putNumber("Pusher Angle", getPusherAngle());
    }
}
