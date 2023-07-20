// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.util.Util;

import frc.robot.Constants;
import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.utils.CTREFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Hood.STATE;
import frc.robot.Ports;

/** Add your docs here. */
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

    public STATE state = STATE.LOCKED;

    public enum STATE {
        HOMING,
        ANGLE,
        PERCENTAGE,
        LOCKED
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
        pusherMotor.setNeutralMode(NeutralMode.Brake);
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

    public synchronized void ResetHook(double angle) {
        hookMotor.setSelectedSensorPosition(
                Conversions.degreesToFalcon(angle, ClimberConstants.HOOK_GEAR_RATIO));
    }

    public synchronized void SetHookAngle(double angle) {
        if (getState() != STATE.ANGLE) {
            setState(STATE.ANGLE);
        }
        angle = Util.clamp(
                angle, ClimberConstants.HOOK_MIN_ANGLE, ClimberConstants.HOOK_MAX_ANGLE);
        periodicIO.hookDemand = angle;
    }

    public synchronized void SetHookPercentage(double power) {
        if (getState() != STATE.PERCENTAGE) {
            setState(STATE.PERCENTAGE);
        }
        periodicIO.hookDemand = power;
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public synchronized void setHookMinimum() {
        SetHookAngle(ClimberConstants.HOOK_MIN_ANGLE);
    }

    public synchronized double getHookAngle() {
        return Conversions.falconToDegrees(periodicIO.hookPosition, ClimberConstants.HOOK_GEAR_RATIO)
    }

    private void updateHookStates() {
        switch (state) {
            case PERCENTAGE:
                periodicIO.hookDemand = Util.clamp(periodicIO.hookDemand, -1.0, 1.0);
                break;
            case ANGLE:
                periodicIO.hookDemand = Util.clamp(periodicIO.hookDemand, ClimberConstants.HOOK_MIN_ANGLE, ClimberConstants.HOOK_MAX_ANGLE);
                break;
            case LOCKED:
                periodicIO.hookDemand = 0.0;
                break;
        }
    }



    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
