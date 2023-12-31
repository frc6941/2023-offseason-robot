// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import org.frcteam1678.lib.math.Conversions;
import org.frcteam6328.utils.TunableNumber;
import org.frcteam6941.swerve.SwerveDrivetrainConstants;
import org.frcteam6941.swerve.SwerveModuleConstants;
import org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits;

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
            DRIVETRAIN_CONSTANTS.setWheelCircumferenceMeters(0.3309432547340153);
            DRIVETRAIN_CONSTANTS.setDeadband(0.01);
            DRIVETRAIN_CONSTANTS.setFreeSpeedMetersPerSecond(4.2);
            DRIVETRAIN_CONSTANTS.setMaxAccelerationMetersPerSecondSquare(61.95850845217319);

            DRIVETRAIN_CONSTANTS.setDriveKP(0.05);
            DRIVETRAIN_CONSTANTS.setDriveKI(0.0);
            DRIVETRAIN_CONSTANTS.setDriveKD(1.0);
            DRIVETRAIN_CONSTANTS.setDriveKV(1023.0 / 20000.0);
            DRIVETRAIN_CONSTANTS.setAngleKP(1.0);
            DRIVETRAIN_CONSTANTS.setAngleKI(0.0);
            DRIVETRAIN_CONSTANTS.setAngleKD(50);
            DRIVETRAIN_CONSTANTS.setAngleKV(1023 * 0.8 / (4096.0 / 4.0));

            DRIVETRAIN_CONSTANTS.setDrivetrainWidthBumped(0.818);
            DRIVETRAIN_CONSTANTS.setDrivetrainWidthFrame(0.7);
            DRIVETRAIN_CONSTANTS.setDrivetrainCenterofRotation(new Translation2d(0, 0));
            DRIVETRAIN_CONSTANTS.setDrivetrainModPositions(new Translation2d[]{
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
                0.69522, 2.3623, 0.19367);

        public static final TrapezoidProfile.Constraints TRANSLATION_CONTROLLER_CONSTRAINT = new TrapezoidProfile.Constraints(
                3.0,
                10.0
        );

        public static final KinematicLimits DRIVETRAIN_UNCAPPED = new KinematicLimits(
                DRIVETRAIN_CONSTANTS.getFreeSpeedMetersPerSecond(),
                DRIVETRAIN_CONSTANTS.getMaxAccelerationMetersPerSecondSquare() * 0.8,
                2000.0);
        public static final KinematicLimits DRIVETRAIN_SMOOTHED = new KinematicLimits(
                DRIVETRAIN_CONSTANTS.getFreeSpeedMetersPerSecond(),
                30.0,
                200.0);
        public static final KinematicLimits DRIVETRAIN_LIMITED = new KinematicLimits(
                2.0,
                DRIVETRAIN_CONSTANTS.getMaxAccelerationMetersPerSecondSquare() * 0.5,
                1200.0);
        public static final KinematicLimits DRIVETRAIN_ROBOT_ORIENTED = new KinematicLimits(
                2.0,
                DRIVETRAIN_CONSTANTS.getMaxAccelerationMetersPerSecondSquare() * 0.5,
                1500.0
        );
        public static final double DRIVETRAIN_MAX_ROTATION_VELOCITY = 300.0;
        public static final double DRIVETRAIN_MAX_ROTATION_ACCELERATION = 720.0;

        public static final SwerveModuleConstants MOD0 = new SwerveModuleConstants();

        static {
            MOD0.setModuleNumber(0);

            MOD0.setDriveMotorPort(Ports.CanId.Rio.DRIVETRAIN_FRONT_LEFT_DRIVE);
            MOD0.setAngleMotorPort(Ports.CanId.Rio.DRIVETRAIN_FRONT_LEFT_STEER);

            MOD0.setAngleOffsetDegreesCCW(289.16015625000006 + 180.0 + 2.021484375 + 357.099609375 - 360.0);
            MOD0.setInvertAngleOutput(true);
            MOD0.setInvertAngleSensorPhase(false);
            MOD0.setOnCanivore(false);
        }

        public static final SwerveModuleConstants MOD1 = new SwerveModuleConstants();

        static {
            MOD1.setModuleNumber(1);

            MOD1.setDriveMotorPort(Ports.CanId.Rio.DRIVETRAIN_FRONT_RIGHT_DRIVE);
            MOD1.setAngleMotorPort(Ports.CanId.Rio.DRIVETRAIN_FRONT_RIGHT_STEER);

            MOD1.setAngleOffsetDegreesCCW(178.76953125 + 180.0 + 2.8125 - 180.0 + 177.62693125);
            MOD1.setInvertAngleOutput(true);
            MOD1.setInvertAngleSensorPhase(false);
            MOD1.setOnCanivore(false);
        }

        public static final SwerveModuleConstants MOD2 = new SwerveModuleConstants();

        static {
            MOD2.setModuleNumber(2);

            MOD2.setDriveMotorPort(Ports.CanId.Rio.DRIVETRAIN_BACK_LEFT_DRIVE);
            MOD2.setAngleMotorPort(Ports.CanId.Rio.DRIVETRAIN_BACK_LEFT_STEER);

            MOD2.setAngleOffsetDegreesCCW(125.41992187500001 + 180.0 + 1.2304 - 180.0 + 179.4727);
            MOD2.setInvertAngleOutput(false);
            MOD2.setInvertAngleSensorPhase(true);
            MOD2.setOnCanivore(false);
        }

        public static final SwerveModuleConstants MOD3 = new SwerveModuleConstants();

        static {
            MOD3.setModuleNumber(3);

            MOD3.setDriveMotorPort(Ports.CanId.Rio.DRIVETRAIN_BACK_RIGHT_DRIVE);
            MOD3.setAngleMotorPort(Ports.CanId.Rio.DRIVETRAIN_BACK_RIGHT_STEER);

            MOD3.setAngleOffsetDegreesCCW(276.50390625 + 180.0 - 180.0 + 174.638671875 + 180.0 - 180.26368125);
            MOD3.setInvertAngleOutput(true);
            MOD3.setInvertAngleSensorPhase(false);
            MOD3.setOnCanivore(false);
        }

    }

    public static final class VisionConstants {
        public static final double HORIZONTAL_FOV = 29.8 * 2; //degrees
        public static final double VERTICAL_FOV = 24.85 * 2; //degrees
        public static final TunableNumber PITCH_DEGREES = new TunableNumber("Camera Pitch", 90 - 57.0);
        public static final TunableNumber HEIGHT_METERS = new TunableNumber("Camera Height", 0.8);
        public static final int[] CAMERA_RESOLUTION = new int[]{960, 720};
        public static final double FRAME_RATE = 22.0;
        public static final double LATENCY = 10.0 / 1000.0;

        public static final double TRACK_MAX_AGE = 10.0;
        public static final double TRACK_MAX_SMOOTHING_TIME = 5.0;
        public static final double TRACK_MAX_DISTANCE = 1.0;

        public static final Translation2d CAMERA_TO_ROBOT_CENTER = new Translation2d(
                0.0, 0.0
        );
        public static final TunableNumber DISTANCE_OFFSET = new TunableNumber("Distance Offset", 0.09);
    }

    public static final class ControllerConstants {
        public static final boolean INVERT_X = false;
        public static final boolean INVERT_Y = false;
        public static final boolean INVERT_R = false;
        public static final double DEADBAND = 0.05;
    }

    public static final class IndexerConstants {
        public static final TunableNumber TUNNEL_KP = new TunableNumber("Feeder Tunnel KP", 0.015);
        public static final TunableNumber TUNNEL_KI = new TunableNumber("Feeder Tunnel KI", 0.001);
        public static final TunableNumber TUNNEL_KD = new TunableNumber("Feeder Tunnel KD", 0.0);
        public static final TunableNumber TUNNEL_KF = new TunableNumber("Feeder Tunnel KF", 0.05);
        public static final double TUNNEL_GEAR_RATIO = 32.0 / 8.0;

        public static final TunableNumber TUNNEL_INDEXING_VELOCITY = new TunableNumber("Feeder Tunnel Indexing Velocity",
                800.0);
        public static final TunableNumber TUNNEL_FEEDING_VELOCITY = new TunableNumber("Feeder Tunnel Feeding Velocity",
                350.0);
        public static final double TUNNEL_REVERSE_VELOCITY = -600.0;

        public static final double EJECTOR_GEAR_RATIO = 14.0 / 40.0;

        public static final double EJECTOR_FAST_VOLTAGE = 12.5;
        public static final double EJECTOR_NORMAL_VOLTAGE = 7.0;
        public static final TunableNumber EJECTOR_FEED_VOLTAGE = new TunableNumber("Feeder Ejector Feed Voltage", 7.0);

        public static final TunableNumber EJECT_CONFIRM_INTERVAL = new TunableNumber("Feeder Ejector Confirm Interval", 0.3);
        public static final TunableNumber NEST_CONFIRM_INTERVAL = new TunableNumber("Feeder Net Confirm Interval", 0.15);
        public static final TunableNumber BOTTOM_CONFIRM_INTERVAL = new TunableNumber("Feeder Net Confirm Interval", 0.05);
    }

    public static class TriggerConstants {
        public static final TunableNumber TRIGGER_KP = new TunableNumber("Feeder Trigger KP", 0.1);
        public static final TunableNumber TRIGGER_KI = new TunableNumber("Feeder Trigger KI", 0.0001);
        public static final TunableNumber TRIGGER_KD = new TunableNumber("Feeder Trigger KD", 0.0);
        public static final TunableNumber TRIGGER_KF = new TunableNumber("Feeder Trigger KF", 0.045);

        public static final double TRIGGER_GEAR_RATIO = 7.0;

        public static final TunableNumber TRIGGER_SLOW_FEEDING_VELOCITY = new TunableNumber("Trigger Slow Feeding Velocity", 300.0);
        public static final TunableNumber TRIGGER_NORMAL_FEEDING_VELOCITY = new TunableNumber("Trigger Normal Feeding Velocity", 600.0);
    }

    public static class ShooterConstants {
        public static final double SHOOTER_GEAR_RATIO = 24.0 / 20.0;

        public static final TunableNumber SHOOTER_KF = new TunableNumber("Shooter KF", 0.05);
        public static final TunableNumber SHOOTER_KP = new TunableNumber("Shooter KP", 0.041);
        public static final TunableNumber SHOOTER_KI = new TunableNumber("Shooter KI", 0.00014);
        public static final TunableNumber SHOOTER_KD = new TunableNumber("Shooter KD", 0.2);
        public static final TunableNumber SHOOTER_IZONE = new TunableNumber("Shooter IZONE", Conversions.RPMToFalcon(300.0, 1.0));
        public static final double SHOOTER_RAMP = 0.1;
        public static final double SHOOTER_ERROR_TOLERANCE = 150.0;
    }

    public static class HoodConstants {
        public static final double HOOD_GEAR_RATIO = (56.0 / 14.0) * (276.0 / 11.0);
        public static final double HOOD_MINIMUM_ANGLE = 10.0;
        public static final double HOOD_MAXIMUM_ANGLE = 35.0;

        public static final double HOOD_KP = 0.75;
        public static final double HOOD_KI = 0.001;
        public static final double HOOD_KD = 0.00;
        public static final double HOOD_KF = 0.05;
        public static final double HOOD_CRUISE_V = 30000.0 * 2.0;
        public static final double HOOD_CRUISE_ACC = 30000.0 * 3.0;
        public static final int HOOD_S_STRENGTH = 2;

        public static final double HOOD_HOMING_CURRENT_THRESHOLD = 12.0;
    }

    public static class ColorSensorConstants {
        public static final double COLOR_SENSOR_RATIO_THRESHOLD = 0.30;
    }

    public static class JudgeConstants {
        public static final double FLYWHEEL_RPM_TOLERANCE = 150.0;
        public static final double BACKBOARD_ANGLE_TOLERANCE = 0.5;
        public static final double DRIVETRAIN_AIM_TOLERANCE_NEAR = 4.50;
        public static final double DRIVETRAIN_AIM_TOLERANCE_FACTOR = (4.50 - 3.00) / (7.0 - 2.0);
    }

    public static class IntakerConstants {
        public static final TunableNumber ROLLING_VOLTAGE = new TunableNumber("Rolling Voltage", 10.0);
        public static final TunableNumber HOPPER_VOLTAGE = new TunableNumber("Hopper Voltage", 12.0);
        public static final double DEPLOY_GEAR_RATIO = 18.0;

        public static final TunableNumber DEPLOY_EXTEND_ANGLE_THRESHOLD = new TunableNumber("Deploy Soft Range", 15.0);
        public static final TunableNumber DEPLOY_EXTEND_ANGLE = new TunableNumber("Deploy Extend Target", 100);
        public static final TunableNumber DEPLOY_CONTRACT_ANGLE = new TunableNumber("Deploy Contract Angle", 15.0);
        public static final TunableNumber DEPLOY_ZEROING_CURRENT = new TunableNumber("Deploy Zeroing Current", 12);
        public static final TunableNumber DEPLOY_ZEROING_VELOCITY = new TunableNumber("Deploy Zeroing Velocity", -0.2);

        public static final TunableNumber DEPLOY_TOUGH_KP = new TunableNumber("Deploy Tough kP", 0.3);
        public static final TunableNumber DEPLOY_TOUGH_KI = new TunableNumber("Deploy Tough kI", 0);
        public static final TunableNumber DEPLOY_TOUGH_KD = new TunableNumber("Deploy Tough kD", 0.0);
        public static final TunableNumber DEPLOY_SOFT_KP = new TunableNumber("Deploy Soft kP", 0.1);
        public static final TunableNumber DEPLOY_SOFT_KI = new TunableNumber("Deploy Soft kI", 0);
        public static final TunableNumber DEPLOY_SOFT_KD = new TunableNumber("Deploy Soft kD", 0);
    }

    public static class ClimberConstants {
        public static final double HOOK_GEAR_RATIO = 64.0;
        public static final double PUSHER_GEAR_RATIO = 64.0;

        public static final double HOOK_MAX_ANGLE = 1200;
        public static final double HOOK_MIN_ANGLE = -5.0;
        public static final double PUSHER_MAX_ANGLE = 5.0;
        public static final double PUSHER_MIN_ANGLE = -720;

        public static final TunableNumber HOOK_KP = new TunableNumber("Hook KP", 0.3);
        public static final TunableNumber HOOK_KI = new TunableNumber("Hook KI", 0.00);
        public static final TunableNumber HOOK_KD = new TunableNumber("Hook KD", 0.0);
        public static final TunableNumber HOOK_KF = new TunableNumber("Hook KF", 0.0);
        public static final double HOOK_CRUISE_V = 50000;
        public static final double HOOK_CRUISE_ACC = 100000;
        public static final int HOOK_S_STRENGTH = 1;

        public static final TunableNumber PUSHER_KP = new TunableNumber("Pusher KP", 0.3);
        public static final TunableNumber PUSHER_KI = new TunableNumber("Pusher KI", 0.00);
        public static final TunableNumber PUSHER_KD = new TunableNumber("Pusher KD", 0.0);
        public static final TunableNumber PUSHER_KF = new TunableNumber("Pusher KF", 0.0);
        public static final double PUSHER_CRUISE_V = 50000;
        public static final double PUSHER_CRUISE_ACC = 100000;
        public static final int PUSHER_S_STRENGTH = 1;

        public static final class AutoClimbSetpoints {
            public static final double HOOK_START_ANGLE = 0;
            public static final double HOOK_READY_ANGLE = 980;
            public static final double HOOK_DEMANDED_ANGLE = 40;
            public static final double HOOK_END_ANGLE = 720.0;
            public static final double HOOK_PUSHER_READY_ANGLE = 800;
            public static final double PUSHER_START_ANGLE = 0;
            public static final double PUSHER_READY_ANGLE = -550;
            public static final double PUSHER_DEMANDED_ANGLE = -90;
        }
    }
}
