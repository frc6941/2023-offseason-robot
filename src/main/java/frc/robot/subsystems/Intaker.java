package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.util.Util;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Ports;
import lombok.Getter;
import lombok.Synchronized;
import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.utils.CTREFactory;

public class Intaker implements Subsystem, Updatable {
    private class PeriodicIO {
        // Inputs
        private double rollerCurrent;
        private double deployCurrent;
        private double hopperCurrent;

        private double rollerVoltage;
        private double deployVoltage;
        private double hopperVoltage;

        // Outputs
        private double rollerDemand;
        private double deployDemand;
        private double hopperDemand;
    }

    private enum States {
        IDLE, INTAKING, REVERSING, REJECTING
    }

    private static Intaker instance;

    public static Intaker getInstance() {
        if (instance == null) {
            instance = new Intaker();
        }
        return instance;
    }

    private final TalonFX roller;
    private final TalonFX deploy;
    private final TalonFX hopper;

    @Getter
    private final PeriodicIO periodicIO = new PeriodicIO();

    private final NetworkTableEntry rollerCurrentEntry;
    private final NetworkTableEntry rollerVoltageEntry;
    private final NetworkTableEntry deployCurrentEntry;
    private final NetworkTableEntry deployVoltageEntry;
    private final NetworkTableEntry hopperCurrentEntry;
    private final NetworkTableEntry hopperVoltageEntry;

    private Intaker() {
        roller = CTREFactory.createDefaultTalonFX(Ports.CanId.Canivore.INTAKE_ROLLER, false);
        deploy = CTREFactory.createDefaultTalonFX(Ports.CanId.Canivore.INTAKE_DEPLOY, false);
        hopper = CTREFactory.createDefaultTalonFX(Ports.CanId.Rio.HOPPER, false);

        roller.setInverted(true);
        deploy.setInverted(true);
        hopper.setInverted(true);

        // Tough PID
        deploy.config_kP(0, Constants.IntakerConstants.DEPLOY_TOUGH_KP.get());
        deploy.config_kI(0, Constants.IntakerConstants.DEPLOY_TOUGH_KI.get());
        deploy.config_kD(0, Constants.IntakerConstants.DEPLOY_TOUGH_KD.get());

        // Soft PID
        deploy.config_kP(1, Constants.IntakerConstants.DEPLOY_SOFT_KP.get());
        deploy.config_kI(1, Constants.IntakerConstants.DEPLOY_SOFT_KI.get());
        deploy.config_kD(1, Constants.IntakerConstants.DEPLOY_SOFT_KD.get());

        deploy.selectProfileSlot(0, 0);
        deploy.setNeutralMode(NeutralMode.Coast);

        roller.changeMotionControlFramePeriod(255);
        roller.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        roller.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);

        hopper.changeMotionControlFramePeriod(255);
        hopper.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        hopper.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);

        if (Constants.TUNING) {
            ShuffleboardTab dataTab = Shuffleboard.getTab(this.getClass().getName());
            rollerCurrentEntry = dataTab.add("Roller Current", periodicIO.rollerCurrent).getEntry();
            rollerVoltageEntry = dataTab.add("Roller Voltage", periodicIO.rollerVoltage).getEntry();

            deployCurrentEntry = dataTab.add("Deploy Current", periodicIO.deployCurrent).getEntry();
            deployVoltageEntry = dataTab.add("Deploy Voltage", periodicIO.deployVoltage).getEntry();

            hopperCurrentEntry = dataTab.add("Hopper Current", periodicIO.hopperCurrent).getEntry();
            hopperVoltageEntry = dataTab.add("Hopper Voltage", periodicIO.hopperVoltage).getEntry();
        }
    }

    public void deploy() {
        periodicIO.deployDemand = Conversions.degreesToFalcon(
                Constants.IntakerConstants.DEPLOY_EXTEND_ANGLE,
                Constants.IntakerConstants.DEPLOY_GEAR_RATIO
        );
    }

    public void resetDeploy() {
        periodicIO.deployDemand = Conversions.degreesToFalcon(
                Constants.IntakerConstants.DEPLOY_CONTRACT_ANGLE,
                Constants.IntakerConstants.DEPLOY_GEAR_RATIO
        );
    }

    public void roll() {
        periodicIO.rollerDemand = Constants.IntakerConstants.ROLLING_VOLTAGE;
        periodicIO.hopperDemand = Constants.IntakerConstants.HOPPER_VOLTAGE;
    }

    public void stopRolling() {
        periodicIO.rollerDemand = 0.0;
        periodicIO.hopperDemand = 0.0;
    }

    @Override
    public void update(double time, double dt) {
        if (isDeployAtSetpoint()) {
            deploy.selectProfileSlot(1, 0);
            return;
        }
        deploy.selectProfileSlot(0, 0);
    }

    public boolean isDeployAtSetpoint() {
        return Util.epsilonEquals(
                Conversions.falconToDegrees(
                        deploy.getSelectedSensorPosition(),
                        Constants.IntakerConstants.DEPLOY_GEAR_RATIO
                ),
                Constants.IntakerConstants.DEPLOY_EXTEND_ANGLE,
                Constants.IntakerConstants.DEPLOY_EXTEND_ANGLE_THRESHOLD
        );
    }

    @Override
    @Synchronized
    public void read(double time, double dt) {
        periodicIO.rollerCurrent = roller.getStatorCurrent();
        periodicIO.rollerVoltage = roller.getMotorOutputVoltage();

        periodicIO.deployCurrent = deploy.getStatorCurrent();
        periodicIO.deployVoltage = deploy.getMotorOutputVoltage();

        periodicIO.hopperCurrent = hopper.getStatorCurrent();
        periodicIO.hopperVoltage = hopper.getMotorOutputVoltage();
    }

    @Override
    public void write(double time, double dt) {
        roller.set(ControlMode.PercentOutput, periodicIO.rollerDemand / 12);
        deploy.set(ControlMode.Position, periodicIO.deployDemand);
        hopper.set(ControlMode.PercentOutput, periodicIO.hopperDemand);
    }

    @Override
    public void stop() {
        roller.set(ControlMode.PercentOutput, 0);
        deploy.set(ControlMode.Position, 0);
        hopper.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void telemetry() {
        if (!Constants.TUNING) return;

        rollerCurrentEntry.setDouble(periodicIO.rollerCurrent);
        rollerVoltageEntry.setDouble(periodicIO.rollerVoltage);

        deployCurrentEntry.setDouble(periodicIO.deployCurrent);
        deployVoltageEntry.setDouble(periodicIO.deployVoltage);

        hopperCurrentEntry.setDouble(periodicIO.hopperCurrent);
        hopperVoltageEntry.setDouble(periodicIO.hopperVoltage);
    }
}
