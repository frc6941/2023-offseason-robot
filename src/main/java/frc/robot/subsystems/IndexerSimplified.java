package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Ports;
import frc.robot.display.OperatorDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.Synchronized;
import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.drivers.BeamBreak;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.utils.CTREFactory;
import org.frcteam6941.utils.TimeDelayedBooleanSimulatable;

import java.util.LinkedList;
import java.util.Queue;

public class IndexerSimplified implements Subsystem, Updatable {
    private class PeriodicIO {
        // Inputs
        public double tunnelVelocity = 0.0;
        public double tunnelCurrent = 0.0;
        public double tunnelVoltage = 0.0;
        public double ejectorVelocity = 0.0;
        public double ejectorCurrent = 0.0;
        public double ejectorVoltage = 0.0;

        // Outputs
        public double tunnelTargetVelocity = 0.0;
        public double ejectorTargetVoltage = 0.0;
    }

    public PeriodicIO periodicIO = new PeriodicIO();

    private static IndexerSimplified instance;

    public static IndexerSimplified getInstance() {
        if (instance == null) {
            instance = new IndexerSimplified();
        }
        return instance;
    }

    private final TalonFX ejector;
    private final TalonFX tunnel;

    @Getter
    private final BeamBreak bottomBeamBreak;
    @Getter
    private final BeamBreak topBeamBreak;

    private final Slot topSlot;
    private final Slot bottomSlot;

    @Setter
    private boolean wantForceEject = false;
    @Setter
    private boolean wantForceReverse = false;
    @Setter
    private boolean wantFeed = false;
    @Setter
    private boolean wantOff = false;
    @Setter
    private boolean fastEject = false;

    private boolean wantIndex = false;
    private boolean wantEject = false;

    @Getter
    private boolean full = false;
    @Getter
    private int ballCount = 0;
    private boolean indexingTopBall = false;
    private boolean indexingBottomBall = false;

    private boolean ejectorReached = false;
    private boolean triggerReached = false;
    private boolean needClear = false;
    private final TimeDelayedBooleanSimulatable ejected = new TimeDelayedBooleanSimulatable();
    private final TimeDelayedBooleanSimulatable triggerNested = new TimeDelayedBooleanSimulatable();

    @Getter
    private State state = State.IDLE;

    private final ShuffleboardTab dataTab;
    private final NetworkTableEntry stateEntry;
    private final NetworkTableEntry ballCountEntry;
    private final NetworkTableEntry tunnelVelocityEntry;
    private final NetworkTableEntry tunnelCurrentEntry;
    private final NetworkTableEntry tunnelVoltageEntry;
    private final NetworkTableEntry tunnelTargetVelocityEntry;
    private final NetworkTableEntry ejectorVelocityEntry;
    private final NetworkTableEntry ejectorCurrentEntry;
    private final NetworkTableEntry ejectorVoltageEntry;
    private final NetworkTableEntry ejectorTargetVoltageEntry;

    private IndexerSimplified() {
        ejector = CTREFactory.createDefaultTalonFX(Ports.CanId.Canivore.INDEXER_EJECTOR, true);
        tunnel = CTREFactory.createDefaultTalonFX(Ports.CanId.Canivore.INDEXER_TUNNEL, true);

        tunnel.config_kP(0, IndexerConstants.TUNNEL_KP.get());
        tunnel.config_kI(0, IndexerConstants.TUNNEL_KI.get());
        tunnel.config_kD(0, IndexerConstants.TUNNEL_KD.get());
        tunnel.config_kF(0, IndexerConstants.TUNNEL_KF.get());
        tunnel.config_IntegralZone(0, 200);
        tunnel.configClosedloopRamp(0.1);

        bottomBeamBreak = new BeamBreak(Ports.AnalogInputId.BOTTOM_BEAM_BREAK_CHANNEL);
        topBeamBreak = new BeamBreak(Ports.AnalogInputId.TOP_BEAM_BREAK_CHANNEL);
        bottomSlot = new Slot();
        topSlot = new Slot();

        if(Constants.TUNING) {
            dataTab = Shuffleboard.getTab("Indexer");
            stateEntry = dataTab.add("State", state.toString()).getEntry();
            ballCountEntry = dataTab.add("Ball Count Stored", getBallCount()).getEntry();
            tunnelVelocityEntry = dataTab.add("Tunnel Velocity", periodicIO.tunnelVelocity).getEntry();
            tunnelCurrentEntry = dataTab.add("Tunnel Current", periodicIO.tunnelCurrent).getEntry();
            tunnelVoltageEntry = dataTab.add("Tunnel Voltage", periodicIO.tunnelVoltage).getEntry();
            tunnelTargetVelocityEntry = dataTab.add("Tunnel Target Velocity", periodicIO.tunnelTargetVelocity).getEntry();
            ejectorVelocityEntry = dataTab.add("Ejector Velocity", periodicIO.ejectorVelocity).getEntry();
            ejectorCurrentEntry = dataTab.add("Ejector Current", periodicIO.ejectorCurrent).getEntry();
            ejectorVoltageEntry = dataTab.add("Ejector Voltage", periodicIO.ejectorVoltage).getEntry();
            ejectorTargetVoltageEntry = dataTab.add("Ejector Target Voltage", periodicIO.ejectorTargetVoltage).getEntry();
        }
    }

    @Synchronized
    public void queueBall(boolean isCorrect) {
        if(isCorrect) {
            if (!topSlot.isOccupied() && !topSlot.isQueued()) {
                topSlot.queueBall(true);
                indexingTopBall = true;
                wantIndex = true;
            } else if (!bottomSlot.isOccupied() && !bottomSlot.isQueued()) {
                bottomSlot.queueBall(true);
                indexingBottomBall = true;
                wantIndex = true;
            }
        } else {
            wantEject = true;
        }
    }

    @Synchronized
    public void clearQueue() {
        topSlot.clear();
        bottomSlot.clear();
    }

    @Synchronized
    public void reset() {
        topSlot.clear();
        bottomSlot.clear();

        wantForceEject = false;
        wantForceReverse = false;
        wantFeed = false;
        wantIndex = false;
        wantEject = false;
        wantOff = false;

        triggerNested.reset();
        ejected.reset();
    }

    /**
     * Handle transitions between states.
     * Transitions are taken according to the "flags" and their relative importance.
     */
    private void handleTransitions() {
        if (wantOff) {
            state = State.IDLE;
            return;
        }
        if (wantForceEject) {
            state = State.FORCE_EJECTING;
            return;
        }

        if (wantForceReverse) {
            state = State.FORCE_REVERSING;
            return;
        }

        if (wantFeed) {
            state = State.FEEDING;
        } else if (wantIndex) {
            state = State.INDEXING;
        } else if (wantEject) {
            state = State.EJECTING;
        } else {
            state = State.IDLE;
        }
    }

    /**
     * Determine indexer setpoints according to the states.
     */
    private void updateIndexerStates() {
        switch (state) {
            case FORCE_EJECTING:
                periodicIO.tunnelTargetVelocity = IndexerConstants.TUNNEL_INDEXING_VELOCITY.get();
                periodicIO.ejectorTargetVoltage = fastEject ? -IndexerConstants.EJECTOR_FAST_VOLTAGE
                        : -IndexerConstants.EJECTOR_NORMAL_VOLTAGE;
                break;
            case FORCE_REVERSING:
                periodicIO.tunnelTargetVelocity = IndexerConstants.TUNNEL_REVERSE_VELOCITY;
                periodicIO.ejectorTargetVoltage = IndexerConstants.EJECTOR_FAST_VOLTAGE;
                break;
            case FEEDING:
                periodicIO.tunnelTargetVelocity = IndexerConstants.TUNNEL_FEEDING_VELOCITY.get();
                periodicIO.ejectorTargetVoltage = IndexerConstants.EJECTOR_FEED_VOLTAGE.get();
                clearQueue();

                if (topBeamBreak.get()) {
                    triggerReached = true;
                }
                break;
            case INDEXING:
                if (indexingTopBall) {
                    if (triggerNested.update(topBeamBreak.get(), IndexerConstants.NEST_CONFIRM_INTERVAL.get())) {
                        indexingTopBall = false;
                        triggerNested.reset();
                        wantIndex = false;
                    }
                } else if (indexingBottomBall) {
                    if (bottomBeamBreak.get()) {
                        indexingBottomBall = false;
                        wantIndex = false;
                    }
                }

                if (indexingTopBall || indexingBottomBall) {
                    periodicIO.tunnelTargetVelocity = IndexerConstants.TUNNEL_INDEXING_VELOCITY.get();
                    periodicIO.ejectorTargetVoltage = IndexerConstants.EJECTOR_FAST_VOLTAGE;
                } else {
                    periodicIO.tunnelTargetVelocity = 0.0;
                    periodicIO.ejectorTargetVoltage = 0.0;
                }
                break;
            case EJECTING:
                periodicIO.tunnelTargetVelocity = IndexerConstants.TUNNEL_INDEXING_VELOCITY.get();
                periodicIO.ejectorTargetVoltage = fastEject ? -IndexerConstants.EJECTOR_FAST_VOLTAGE
                        : -IndexerConstants.EJECTOR_NORMAL_VOLTAGE;

                if (bottomBeamBreak.get()) {
                    ejectorReached = true;
                }

                if (ejected.update(
                        (ejectorReached && !bottomBeamBreak.get()),
                        IndexerConstants.EJECT_CONFIRM_INTERVAL.get())) {
                    System.out.println("Ejected!");
                    ejected.reset();
                    ejectorReached = false;
                    wantEject = false;
                }

                break;
            case IDLE:
            default:
                periodicIO.tunnelTargetVelocity = 0.0;
                periodicIO.ejectorTargetVoltage = 0.0;
                break;
        }
    }

    public void updateBallCounter() {
        if (topSlot.isOccupied() && bottomSlot.isOccupied()) {
            ballCount = 2;
        } else if (topSlot.isOccupied() || bottomSlot.isOccupied()) {
            ballCount = 1;
        } else {
            ballCount = 0;
        }
    }

    @Override
    public void read(double time, double dt) {
        periodicIO.tunnelVelocity = Conversions.falconToRPM(tunnel.getSelectedSensorVelocity(),
                IndexerConstants.TUNNEL_GEAR_RATIO);
        periodicIO.tunnelCurrent = tunnel.getSupplyCurrent();
        periodicIO.tunnelVoltage = tunnel.getMotorOutputVoltage();

        periodicIO.ejectorVelocity = Conversions.falconToRPM(ejector.getSelectedSensorVelocity(),
                IndexerConstants.EJECTOR_GEAR_RATIO);
        periodicIO.ejectorCurrent = ejector.getSupplyCurrent();
        periodicIO.ejectorVoltage = ejector.getMotorOutputVoltage();

        ejected.updateTime(time);
        triggerNested.updateTime(time);
    }

    @Override
    public void update(double time, double dt) {
        topBeamBreak.update();
        bottomBeamBreak.update();
        topSlot.update(topBeamBreak.get());
        bottomSlot.update(bottomBeamBreak.get() && topSlot.isOccupied());

        full = ballCount == 2;

        handleTransitions();
        updateIndexerStates();
        updateBallCounter();
        handleTransitions(); // make sure state change in updateIndexerStates() have effect

        if (!RobotState.isDisabled()) return;

        if(IndexerConstants.TUNNEL_KP.hasChanged()) {
            System.out.println("Configuring Tunnel KP!");
            tunnel.config_kP(0, IndexerConstants.TUNNEL_KP.get());
        }
        if(IndexerConstants.TUNNEL_KI.hasChanged()) {
            System.out.println("Configuring Tunnel KI!");
            tunnel.config_kI(0, IndexerConstants.TUNNEL_KI.get());
        }
        if(IndexerConstants.TUNNEL_KD.hasChanged()) {
            System.out.println("Configuring Tunnel KD!");
            tunnel.config_kD(0, IndexerConstants.TUNNEL_KD.get());
        }
        if(IndexerConstants.TUNNEL_KF.hasChanged()) {
            System.out.println("Configuring Tunnel KF!");
            tunnel.config_kF(0, IndexerConstants.TUNNEL_KF.get());
        }
    }

    @Override
    public void write(double time, double dt) {
        ejector.set(ControlMode.PercentOutput, periodicIO.ejectorTargetVoltage / 12.0);
        if (periodicIO.tunnelTargetVelocity == 0.0) {
            tunnel.set(ControlMode.PercentOutput, 0.0); // Open loop, prevent stuck
            return;
        }
        tunnel.set(ControlMode.PercentOutput,
                Conversions.RPMToFalcon(periodicIO.tunnelTargetVelocity, IndexerConstants.TUNNEL_GEAR_RATIO));
    }

    @Override
    public void telemetry() {
        if(Constants.TUNING) {
            stateEntry.setString(state.toString());
            ballCountEntry.setDouble(getBallCount());
            tunnelCurrentEntry.setDouble(periodicIO.tunnelCurrent);
            tunnelVelocityEntry.setDouble(periodicIO.tunnelVelocity);
            tunnelVoltageEntry.setDouble(periodicIO.tunnelVoltage);
            tunnelTargetVelocityEntry.setDouble(periodicIO.tunnelTargetVelocity);
            ejectorCurrentEntry.setDouble(periodicIO.ejectorCurrent);
            ejectorVelocityEntry.setDouble(periodicIO.ejectorVelocity);
            ejectorVoltageEntry.setDouble(periodicIO.ejectorVoltage);
            ejectorTargetVoltageEntry.setDouble(periodicIO.ejectorTargetVoltage);
        }

        OperatorDashboard feedback = OperatorDashboard.getInstance();
        feedback.getBallFull().setBoolean(getBallCount() == 2);
        feedback.getBallPathIndexing().setBoolean(state == State.INDEXING);
        feedback.getBallPathEjecting().setBoolean(state == State.EJECTING);
        feedback.getBallPathFeeding().setBoolean(state == State.FEEDING);
    }

    @Getter
    @RequiredArgsConstructor
    private static class Slot {
        private boolean occupied;
        private boolean queued;
        private boolean correct;

        public void queueBall(boolean isCorrect) {
            queued = true;
            correct = isCorrect;
        }

        public void update(boolean status) {
            occupied = status && queued;
        }

        public void clear() {
            queued = false;
        }
    }

    /**
     * States of the Indexer.
     * <p>
     * FORCE_EJECTING: send all cargo to the ejector regardless of their color.
     * FORCE_REVERSING: send all cargo to the hopper regardless of their color.
     * FEEDING: send cargo to the trigger.
     * INDEXING: index cargo to occupy the two available slots, and go idle afterward.
     * EJECTING: send wrong cargo to the ejector, and go idle afterward.
     * IDLE: stay in place.
     */
    public enum State {
        FORCE_EJECTING, FORCE_REVERSING, FEEDING, INDEXING, EJECTING, IDLE
    }
}
