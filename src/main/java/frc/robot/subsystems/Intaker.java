package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team254.lib.util.Util;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Ports;
import lombok.Getter;
import lombok.Synchronized;
import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.drivers.BeamBreak;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.utils.CTREFactory;

public class Intaker implements Subsystem, Updatable {
    private static class PeriodicIO {
        // Inputs
        public double rollerCurrent = 0.0;
        public double deployCurrent = 0.0;
        private double rollerVoltage = 0.0;
        private double deployVoltage = 0.0;
        private double hopperVoltage = 0.0;
        private double deployAngle = 0.0;

        // Outputs
        private double rollerDemand = 0.0;
        private double deployDemand = 0.0;
        private double hopperDemand = 0.0;
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
    private final TalonSRX hopper;
    private final BeamBreak entranceDetector;

    private final PeriodicIO periodicIO = new PeriodicIO();

    @Getter
    private boolean homed = false;
    private boolean sawBall = false;

    private final NetworkTableEntry rollerCurrentEntry;
    private final NetworkTableEntry rollerVoltageEntry;
    private final NetworkTableEntry deployCurrentEntry;
    private final NetworkTableEntry deployVoltageEntry;
    private final NetworkTableEntry hopperVoltageEntry;
    private final NetworkTableEntry deployDemandEntry;

    private Intaker() {
        roller = CTREFactory.createDefaultTalonFX(Ports.CanId.Canivore.INTAKE_ROLLER, false);
        deploy = CTREFactory.createDefaultTalonFX(Ports.CanId.Canivore.INTAKE_DEPLOY, false);
        hopper = CTREFactory.createDefaultTalonSRX(Ports.CanId.Rio.HOPPER);

        roller.setInverted(true);
        deploy.setInverted(false);
        hopper.setInverted(true);

        // Tough PID
        deploy.config_kP(0, Constants.IntakerConstants.DEPLOY_TOUGH_KP.get());
        deploy.config_kI(0, Constants.IntakerConstants.DEPLOY_TOUGH_KI.get());
        deploy.config_kD(0, Constants.IntakerConstants.DEPLOY_TOUGH_KD.get());

        // Soft PID
        deploy.config_kP(1, Constants.IntakerConstants.DEPLOY_SOFT_KP.get());
        deploy.config_kI(1, Constants.IntakerConstants.DEPLOY_SOFT_KI.get());
        deploy.config_kD(1, Constants.IntakerConstants.DEPLOY_SOFT_KD.get());

        deploy.configMotionSCurveStrength(2);
        deploy.configMotionCruiseVelocity(12000);
        deploy.configMotionAcceleration(12000);

        deploy.selectProfileSlot(0, 0);

        deploy.setNeutralMode(NeutralMode.Brake);

        roller.changeMotionControlFramePeriod(255);
        roller.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        roller.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);

        hopper.changeMotionControlFramePeriod(255);
        hopper.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        hopper.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        entranceDetector = new BeamBreak(Ports.AnalogInputId.ENTRANCE_BEAM_BREAK_CHANNEL);

        if (Constants.TUNING) {
            ShuffleboardTab dataTab = Shuffleboard.getTab("Intaker");
            rollerCurrentEntry = dataTab.add("Roller Current", periodicIO.rollerCurrent).getEntry();
            rollerVoltageEntry = dataTab.add("Roller Voltage", periodicIO.rollerVoltage).getEntry();

            deployCurrentEntry = dataTab.add("Deploy Current", periodicIO.deployCurrent).getEntry();
            deployVoltageEntry = dataTab.add("Deploy Voltage", periodicIO.deployVoltage).getEntry();
            deployDemandEntry = dataTab.add("Deploy Demand", periodicIO.deployDemand).getEntry();

            hopperVoltageEntry = dataTab.add("Hopper Voltage", periodicIO.hopperVoltage).getEntry();
        }
    }

    public void deploy() {
        periodicIO.deployDemand = Conversions.degreesToFalcon(
                Constants.IntakerConstants.DEPLOY_EXTEND_ANGLE.get(),
                Constants.IntakerConstants.DEPLOY_GEAR_RATIO
        );
    }

    public void contract() {
        periodicIO.deployDemand = Conversions.degreesToFalcon(
                Constants.IntakerConstants.DEPLOY_CONTRACT_ANGLE.get(),
                Constants.IntakerConstants.DEPLOY_GEAR_RATIO
        );
    }

    public void roll(double rollerVoltage, double hopperVoltage) {
        periodicIO.rollerDemand = -rollerVoltage;
        periodicIO.hopperDemand = hopperVoltage;
    }

    public void stopRolling() {
        roll(0.0, 0.0);
    }

    public boolean isDeployAtSetpoint() {
        return Util.epsilonEquals(
                periodicIO.deployAngle,
                Constants.IntakerConstants.DEPLOY_EXTEND_ANGLE.get(),
                Constants.IntakerConstants.DEPLOY_EXTEND_ANGLE_THRESHOLD.get()
        );
    }

    public void home(double angle) {
        deploy.setSelectedSensorPosition(
                Conversions.degreesToFalcon(
                        angle,
                        Constants.IntakerConstants.DEPLOY_GEAR_RATIO
                )
        );
        homed = true;
    }

    public boolean seesBall() {
        return entranceDetector.get();
    }

    public boolean seesNewBall() {
        boolean newBall = seesBall() && !sawBall;
        sawBall = seesBall();
        return newBall;
    }

    @Override
    @Synchronized
    public void read(double time, double dt) {
        periodicIO.rollerCurrent = roller.getSupplyCurrent();
        periodicIO.rollerVoltage = roller.getMotorOutputVoltage();

        periodicIO.deployCurrent = deploy.getSupplyCurrent();
        periodicIO.deployVoltage = deploy.getMotorOutputVoltage();

        periodicIO.hopperVoltage = hopper.getMotorOutputVoltage();

        periodicIO.deployAngle = Conversions.falconToDegrees(
                deploy.getSelectedSensorPosition(),
                Constants.IntakerConstants.DEPLOY_GEAR_RATIO
        );
    }

    @Override
    public void update(double time, double dt) {
        if (Constants.IntakerConstants.DEPLOY_TOUGH_KP.hasChanged()) {
            deploy.config_kP(0, Constants.IntakerConstants.DEPLOY_TOUGH_KP.get());
        }
        if (Constants.IntakerConstants.DEPLOY_TOUGH_KI.hasChanged()) {
            deploy.config_kI(0, Constants.IntakerConstants.DEPLOY_TOUGH_KI.get());
        }
        if (Constants.IntakerConstants.DEPLOY_TOUGH_KD.hasChanged()) {
            deploy.config_kD(0, Constants.IntakerConstants.DEPLOY_TOUGH_KD.get());
        }
        if (Constants.IntakerConstants.DEPLOY_SOFT_KP.hasChanged()) {
            deploy.config_kP(1, Constants.IntakerConstants.DEPLOY_SOFT_KP.get());
        }
        if (Constants.IntakerConstants.DEPLOY_SOFT_KI.hasChanged()) {
            deploy.config_kP(1, Constants.IntakerConstants.DEPLOY_SOFT_KI.get());
        }
        if (Constants.IntakerConstants.DEPLOY_SOFT_KD.hasChanged()) {
            deploy.config_kD(1, Constants.IntakerConstants.DEPLOY_SOFT_KD.get());
        }
        if (!homed) {
            if (periodicIO.deployCurrent >
                    Constants.IntakerConstants.DEPLOY_ZEROING_CURRENT.get()) home(0.0);
            return;
        }
//        if (isDeployAtSetpoint()) {
//            deploy.selectProfileSlot(1, 0);
//            return;
//        }
        deploy.selectProfileSlot(0, 0);
    }

    @Override
    public void write(double time, double dt) {
        roller.set(ControlMode.PercentOutput, periodicIO.rollerDemand / 12);
        if (!homed) {
            deploy.set(ControlMode.PercentOutput, Constants.IntakerConstants.DEPLOY_ZEROING_VELOCITY.get());
        } else {
            deploy.set(ControlMode.MotionMagic, periodicIO.deployDemand);
        }

        hopper.set(ControlMode.PercentOutput, periodicIO.hopperDemand);
    }

    @Override
    public void telemetry() {
        if (!Constants.TUNING) return;

        rollerCurrentEntry.setDouble(periodicIO.rollerCurrent);
        rollerVoltageEntry.setDouble(periodicIO.rollerVoltage);

        deployCurrentEntry.setDouble(periodicIO.deployCurrent);
        deployVoltageEntry.setDouble(periodicIO.deployVoltage);

        hopperVoltageEntry.setDouble(periodicIO.hopperVoltage);
    }
}
