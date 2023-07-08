// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.frcteam1678.lib.math.Conversions;
import org.frcteam6328.utils.TunableNumber;
import org.frcteam6941.swerve.SwerveDrivetrainConstants;
import org.frcteam6941.swerve.SwerveModuleConstants;
import org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean TUNING = true;

    public static final boolean IS_REAL = TimedRobot.isReal();

    // FMS Related Information
    public static final class FMS {
        public static Alliance ALLIANCE() {
            return DriverStation.getAlliance();
        }
    }

    // Looper Configurations
    public static final double LOOPER_DT = 1.0 / 150.0;

    public static final class SwerveConstants {
        public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants();
        static {
            DRIVETRAIN_CONSTANTS.setDriveMotor(DCMotor.getFalcon500(1));
            DRIVETRAIN_CONSTANTS.setAngleMotor(DCMotor.getVex775Pro(1));

            DRIVETRAIN_CONSTANTS.setAngleGearRatio((56.0 / 6.0) * (60.0 / 10.0));
            DRIVETRAIN_CONSTANTS.setDriveGearRatio(7.0);
            DRIVETRAIN_CONSTANTS.setWheelCircumferenceMeters(Math.PI * Units.inchesToMeters(4.01));
            DRIVETRAIN_CONSTANTS.setDeadband(0.01);
            DRIVETRAIN_CONSTANTS.setFreeSpeedMetersPerSecond(4.0);

            DRIVETRAIN_CONSTANTS.setDriveKP(0.07);
            DRIVETRAIN_CONSTANTS.setDriveKI(0.0);
            DRIVETRAIN_CONSTANTS.setDriveKD(1.0);
            DRIVETRAIN_CONSTANTS.setDriveKV(1023.0 / 54717.0);
            DRIVETRAIN_CONSTANTS.setAngleKP(1.0);
            DRIVETRAIN_CONSTANTS.setAngleKI(0.0);
            DRIVETRAIN_CONSTANTS.setAngleKD(50);
            DRIVETRAIN_CONSTANTS.setAngleKV(1023 * 0.8 / (4096.0 / 4.0));

            DRIVETRAIN_CONSTANTS.setDrivetrainWidthBumped(0.818);
            DRIVETRAIN_CONSTANTS.setDrivetrainWidthFrame(0.7);
            DRIVETRAIN_CONSTANTS.setDrivetrainCenterofRotation(new Translation2d(0, 0));
            DRIVETRAIN_CONSTANTS.setDrivetrainModPositions(new Translation2d[] {
                    new Translation2d(DRIVETRAIN_CONSTANTS.getDrivetrainWidthFrame() / 2.0,
                            DRIVETRAIN_CONSTANTS.getDrivetrainWidthFrame() / 2.0),
                    new Translation2d(DRIVETRAIN_CONSTANTS.getDrivetrainWidthFrame() / 2.0,
                            -DRIVETRAIN_CONSTANTS.getDrivetrainWidthFrame() / 2.0),
                    new Translation2d(-DRIVETRAIN_CONSTANTS.getDrivetrainWidthFrame() / 2.0,
                            DRIVETRAIN_CONSTANTS.getDrivetrainWidthFrame() / 2.0),
                    new Translation2d(-DRIVETRAIN_CONSTANTS.getDrivetrainWidthFrame() / 2.0,
                            -DRIVETRAIN_CONSTANTS.getDrivetrainWidthFrame() / 2.0)
            });
            DRIVETRAIN_CONSTANTS.setDrivetrainKinematics(
                    new SwerveDriveKinematics(
                            DRIVETRAIN_CONSTANTS.getDrivetrainModPositions()
                    )
            );
        }

        public static final double DRIVETRAIN_HEADING_CONTROLLER_KP = 0.07;
        public static final double DRIVETRAIN_HEADING_CONTROLLER_KI = 0.006;
        public static final double DRIVETRAIN_HEADING_CONTROLLER_KD = 0.001;

        public static final SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(
                0.60757, 7.6216, 0.71241);

        public static final TrapezoidProfile.Constraints TRANSLATION_CONTROLLER_CONSTRAINT = new TrapezoidProfile.Constraints(
                3.0,
                10.0
        );

        public static final KinematicLimits DRIVETRAIN_UNCAPPED = new KinematicLimits(
                DRIVETRAIN_CONSTANTS.getFreeSpeedMetersPerSecond(),
                Double.POSITIVE_INFINITY,
                900.0);
        public static final KinematicLimits DRIVETRAIN_SMOOTHED = new KinematicLimits(
                DRIVETRAIN_CONSTANTS.getFreeSpeedMetersPerSecond(),
                15.0,
                720.0);
        public static final KinematicLimits DRIVETRAIN_LIMITED = new KinematicLimits(
                2.0,
                7.0,
                500.0);

        public static final SwerveModuleConstants MOD0 = new SwerveModuleConstants();
        static {
            MOD0.setModuleNumber(0);

            MOD0.setDriveMotorPort(Ports.CanId.Rio.DRIVETRAIN_FRONT_LEFT_DRIVE);
            MOD0.setAngleMotorPort(Ports.CanId.Rio.DRIVETRAIN_FRONT_LEFT_STEER);

            MOD0.setAngleOffsetDegreesCCW(289.16015625000006 + 180.0);
            MOD0.setInvertAngleOutput(true);
            MOD0.setInvertAngleSensorPhase(false);
            MOD0.setOnCanivore(false);
        }

        public static final SwerveModuleConstants MOD1 = new SwerveModuleConstants();
        static {
            MOD1.setModuleNumber(1);

            MOD1.setDriveMotorPort(Ports.CanId.Rio.DRIVETRAIN_FRONT_RIGHT_DRIVE);
            MOD1.setAngleMotorPort(Ports.CanId.Rio.DRIVETRAIN_FRONT_RIGHT_STEER);

            MOD1.setAngleOffsetDegreesCCW(178.76953125 + 180.0);
            MOD1.setInvertAngleOutput(true);
            MOD1.setInvertAngleSensorPhase(false);
            MOD1.setOnCanivore(false);
        }

        public static final SwerveModuleConstants MOD2 = new SwerveModuleConstants();
        static {
            MOD2.setModuleNumber(2);

            MOD2.setDriveMotorPort(Ports.CanId.Rio.DRIVETRAIN_BACK_LEFT_DRIVE);
            MOD2.setAngleMotorPort(Ports.CanId.Rio.DRIVETRAIN_BACK_LEFT_STEER);

            MOD2.setAngleOffsetDegreesCCW(125.41992187500001 + 180.0);
            MOD2.setInvertAngleOutput(false);
            MOD2.setInvertAngleSensorPhase(true);
            MOD2.setOnCanivore(false);
        }

        public static final SwerveModuleConstants MOD3 = new SwerveModuleConstants();
        static {
            MOD3.setModuleNumber(3);

            MOD3.setDriveMotorPort(Ports.CanId.Rio.DRIVETRAIN_BACK_RIGHT_DRIVE);
            MOD3.setAngleMotorPort(Ports.CanId.Rio.DRIVETRAIN_BACK_RIGHT_STEER);

            MOD3.setAngleOffsetDegreesCCW(276.50390625 + 180.0);
            MOD3.setInvertAngleOutput(true);
            MOD3.setInvertAngleSensorPhase(false);
            MOD3.setOnCanivore(false);
        }

    }

    public static final class VisionConstants {
        public static final double FOV_DEGREES = 90.0;
        public static final double PITCH_DEGREES = 54.0;
        public static final double HEIGHT_METERS = 0.60;
        public static final int[] CAMERA_RESOLUTION = new int[] { 640, 480 };
        public static final double FRAME_RATE = 90.0;
        public static final double LATENCY = 10.0 / 1000.0;

        public static final double TRACK_MAX_AGE = 10.0;
        public static final double TRACK_MAX_SMOOTHING_TIME = 1.5;
        public static final double TRACK_MAX_DISTANCE = 8.0;
    }

    public static final class ControllerConstants {
        public static final boolean INVERT_X = false;
        public static final boolean INVERT_Y = false;
        public static final boolean INVERT_R = false;
        public static final double DEADBAND = 0.05;
    }

    public static final class IndexerConstants {
        public static final TunableNumber TUNNEL_KP = new TunableNumber("Feeder Tunnel KP", 0.01);
        public static final TunableNumber TUNNEL_KI = new TunableNumber("Feeder Tunnel KI", 0.001);
        public static final TunableNumber TUNNEL_KD = new TunableNumber("Feeder Tunnel KD", 0.0);
        public static final TunableNumber TUNNEL_KF = new TunableNumber("Feeder Tunnel KF", 0.045);
        public static final double TUNNEL_GEAR_RATIO = 32.0 / 8.0;

        public static final TunableNumber TUNNEL_INDEXING_VELOCITY = new TunableNumber("Feeder Tunnel Indexing Velocity",
                640.0);
        public static final TunableNumber TUNNEL_FEEDING_VELOCITY = new TunableNumber("Feeder Tunnel Indexing Velocity",
                500.0);
        public static final double TUNNEL_REVERSE_VELOCITY = -300.0;

        public static final double EJECTOR_GEAR_RATIO = 14.0 / 40.0;

        public static final double EJECTOR_FAST_VOLTAGE = 12.0;
        public static final double EJECTOR_NORMAL_VOLTAGE = 8.0;
        public static final TunableNumber EJECTOR_FEED_VOLTAGE = new TunableNumber("Feeder Ejector Feed Voltage", 7.0);

        public static final TunableNumber EJECT_CONFIRM_INTERVAL = new TunableNumber("Feeder Ejector Confirm Interval", 0.15);
        public static final TunableNumber NEST_CONFIRM_INTERVAL = new TunableNumber("Feeder Net Confirm Interval", 0.1);
        public static final TunableNumber FEED_CONFIRM_INTERVAL = new TunableNumber("Feeder Feed Confirm Interval", 0.1);
        public static final TunableNumber CLEAR_CONFIRM_INTERVAL = new TunableNumber("Feeder Clear Confirm Interval", 0.1);
    }

    public static class TriggerConstants {
        public static final TunableNumber TRIGGER_KP = new TunableNumber("Feeder Trigger KP", 0.1);
        public static final TunableNumber TRIGGER_KI = new TunableNumber("Feeder Trigger KI", 0.0);
        public static final TunableNumber TRIGGER_KD = new TunableNumber("Feeder Trigger KD", 0.0);
        public static final TunableNumber TRIGGER_KF = new TunableNumber("Feeder Trigger KF", 0.0);
        public static final TunableNumber TRIGGER_LOCK_KP = new TunableNumber("Feeder Trigger Lock KP", 0.5);
        public static final TunableNumber TRIGGER_LOCK_KI = new TunableNumber("Feeder Trigger Lock KI", 0.0);
        public static final TunableNumber TRIGGER_LOCK_KD = new TunableNumber("Feeder Trigger Lock KD", 0.0);

        public static final double TRIGGER_GEAR_RATIO = 7.0;

        public static final TunableNumber TRIGGER_SLOW_FEEDING_VELOCITY = new TunableNumber("Trigger Slow Feeding Velocity", 300.0);
        public static final TunableNumber TRIGGER_NORMAL_FEEDING_VELOCITY = new TunableNumber("Trigger Normal Feeding Velocity", 500.0);
    }

    public static class ShooterConstants {
        public static final double SHOOTER_GEAR_RATIO = 24.0 / 20.0;
        public static final double SHOOTER_MAX_FREE_SPEED_RPM = 6380.0;

        public static final TunableNumber SHOOTER_KF = new TunableNumber("Shooter KF", 1024.0 / Conversions.RPMToFalcon(SHOOTER_MAX_FREE_SPEED_RPM, 1.0));
        public static final TunableNumber SHOOTER_KP = new TunableNumber("Shooter KP", 0.15);
        public static final TunableNumber SHOOTER_KI = new TunableNumber("Shooter KI", 0.001);
        public static final TunableNumber SHOOTER_KD = new TunableNumber("Shooter KD", 1.5);
        public static final TunableNumber SHOOTER_IZONE = new TunableNumber("Shooter IZONE",Conversions.RPMToFalcon(150, 1.0));
        public static final double SHOOTER_RAMP = 0.25;
        public static final double SHOOTER_ERROR_TOLERANCE = 150.0;
    }

    public static class HoodConstants {
        public static final double HOOD_GEAR_RATIO = (56.0 / 14.0) * (276.0 / 11.0);
        public static final double HOOD_MINIMUM_ANGLE = 7.0;
        public static final double HOOD_MAXIMUM_ANGLE = 30.0;

        public static final double HOOD_KP = 0.75;
        public static final double HOOD_KI = 0.001;
        public static final double HOOD_KD = 0.00;
        public static final double HOOD_KF = 0.05;
        public static final double HOOD_CRUISE_V = 30000.0 * 2.0;
        public static final double HOOD_CRUISE_ACC = 30000.0 * 3.0;
        public static final int HOOD_S_STRENGTH = 2;

        public static final double HOOD_HOMING_CURRENT_THRESHOLD = 8.0;
    }

    public static class ColorSensorConstants {
        public static final double COLOR_SENSOR_RATIO_THRESHOLD = 0.30;
    }

    public static class JudgeConstants {
        public static final double FLYWHEEL_RPM_TOLERANCE = 150.0;
        public static final double BACKBOARD_ANGLE_TOLERANCE = 1.0;
        public static final double DRIVETRAIN_AIM_TOLERANCE = 3.0;
    }

    public static class IntakerConstants {
        public static final double ROLLING_VOLTAGE = 0.0;
        public static final double DEPLOY_GEAR_RATIO = 18;
        public static final double DEPLOY_EXTEND_ANGLE_THRESHOLD = 20;
        public static final double DEPLOY_EXTEND_ANGLE = 100;
        public static final double DEPLOY_CONTRACT_ANGLE = 0;
        public static final TunableNumber DEPLOY_TOUGH_KP = new TunableNumber("Tough kP", 0.0);
        public static final TunableNumber DEPLOY_TOUGH_KI = new TunableNumber("Tough kI", 0.0);
        public static final TunableNumber DEPLOY_TOUGH_KD = new TunableNumber("Tough kD", 0.0);
        public static final TunableNumber DEPLOY_SOFT_KP = new TunableNumber("Soft kP", 0.0);
        public static final TunableNumber DEPLOY_SOFT_KI = new TunableNumber("Soft kI", 0.0);
        public static final TunableNumber DEPLOY_SOFT_KD = new TunableNumber("Soft kD", 0.0);
    }
}
