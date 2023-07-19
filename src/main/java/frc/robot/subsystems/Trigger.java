package frc.robot.subsystems;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.utils.CTREFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Ports;
import frc.robot.Constants.TriggerConstants;
import lombok.Getter;

public class Trigger implements Updatable, Subsystem {
    public static class PeriodicIO {
        // INPUTS
        public double triggerVelocity = 0.0;
        public double triggerCurrent = 0.0;
        public double triggerVoltage = 0.0;
        public double triggerPosition = 0.0;

        // OUTPUTS
        public double triggerDemand = 0.0;
        public boolean triggerNeedLock = false;
    }
    private final PeriodicIO periodicIO = new PeriodicIO();

    private final TalonFX trigger;

    private Double lockPositionRecord = null;

    private static Trigger instance;
    @Getter
    private State state = State.LOCK;

    public static Trigger getInstance() {
        if (instance == null) {
            instance = new Trigger();
        }
        return instance;
    }

    private Trigger() {
        trigger = CTREFactory.createDefaultTalonFX(Ports.CanId.Canivore.TRIGGER, false);

        trigger.config_kP(0, TriggerConstants.TRIGGER_KP.get());
        trigger.config_kI(0, TriggerConstants.TRIGGER_KI.get());
        trigger.config_kD(0, TriggerConstants.TRIGGER_KD.get());
        trigger.config_kF(0, TriggerConstants.TRIGGER_KF.get());

        trigger.config_kP(1, TriggerConstants.TRIGGER_LOCK_KP.get());
        trigger.config_kI(1, TriggerConstants.TRIGGER_LOCK_KI.get());
        trigger.config_kD(1, TriggerConstants.TRIGGER_LOCK_KD.get());
    }

    private void updateTriggerStates() {
        switch(state) {
            case SLOW_FEED:
                periodicIO.triggerDemand = TriggerConstants.TRIGGER_SLOW_FEEDING_VELOCITY.get();
                periodicIO.triggerNeedLock = false;
                break;
            case SLOW_REVERSE:
                periodicIO.triggerDemand = -TriggerConstants.TRIGGER_SLOW_FEEDING_VELOCITY.get();
                periodicIO.triggerNeedLock = false;
                break;
            case FEED:
                periodicIO.triggerDemand = TriggerConstants.TRIGGER_NORMAL_FEEDING_VELOCITY.get();
                periodicIO.triggerNeedLock = false;
                break;
            case REVERSE:
                periodicIO.triggerDemand = -TriggerConstants.TRIGGER_NORMAL_FEEDING_VELOCITY.get();
                periodicIO.triggerNeedLock = false;
                break;
            case LOCK:
                periodicIO.triggerDemand = -100.0;
                periodicIO.triggerNeedLock = false;
                break;
            case IDLE:
            default:
                periodicIO.triggerDemand = 0.0;
                periodicIO.triggerNeedLock = false;
                break;
        }
    }

    @Override
    public void read(double time, double dt) {
        periodicIO.triggerCurrent = trigger.getSupplyCurrent();
        periodicIO.triggerVelocity = Conversions.falconToRPM(trigger.getSelectedSensorVelocity(), TriggerConstants.TRIGGER_GEAR_RATIO);
        periodicIO.triggerVoltage = trigger.getMotorOutputVoltage();
        periodicIO.triggerPosition = trigger.getSelectedSensorPosition();
    }

    @Override
    public void update(double time, double dt) {
        updateTriggerStates();
        if(periodicIO.triggerNeedLock) {
            if(lockPositionRecord == null) {
                lockPositionRecord = periodicIO.triggerPosition;
            }
        } else {
            lockPositionRecord = null;
        }
    }

    @Override
    public void write(double time, double dt) {
        if (periodicIO.triggerDemand != 0.0) {
            trigger.selectProfileSlot(0, 0);
            trigger.set(
                    ControlMode.Velocity,
                    Conversions.RPMToFalcon(periodicIO.triggerDemand, TriggerConstants.TRIGGER_GEAR_RATIO)
            );
            return;
        }


        if (!periodicIO.triggerNeedLock) {
            trigger.selectProfileSlot(0, 0);
            trigger.set(ControlMode.PercentOutput, 0.0);
            return;
        }

        trigger.selectProfileSlot(1, 0);
        trigger.set(ControlMode.Position, lockPositionRecord != null ? lockPositionRecord : periodicIO.triggerPosition);
    }

    public enum State {
        LOCK, IDLE, SLOW_FEED, FEED, SLOW_REVERSE, REVERSE
    }

    public void lock() {
        state = State.LOCK;
    }

    public void idle() {
        state = State.IDLE;
    }

    public void feed(boolean slow) {
        state = slow ? State.SLOW_FEED : State.FEED;
    }

    public void feed() {
        feed(false);
    }

    public void reverse(boolean slow) {
        state = slow ? State.SLOW_REVERSE : State.REVERSE;
    }
}
