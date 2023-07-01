// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frcteam6941.swerve.SwerveDrivetrainConstants;
import org.frcteam6941.swerve.SwerveModuleConstants;
import org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
    public static final boolean AUTO_TUNING = false;

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

            DRIVETRAIN_CONSTANTS.setDriveKP(0.1);
            DRIVETRAIN_CONSTANTS.setDriveKI(0.0);
            DRIVETRAIN_CONSTANTS.setDriveKD(0.0);
            DRIVETRAIN_CONSTANTS.setDriveKV(1023.0 / 54717.0 * (3.9 / 1.5));
            DRIVETRAIN_CONSTANTS.setAngleKP(1.5);
            DRIVETRAIN_CONSTANTS.setAngleKI(0.0);
            DRIVETRAIN_CONSTANTS.setAngleKD(60);
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
            DRIVETRAIN_CONSTANTS.setDrivetrainKinematics(new SwerveDriveKinematics(
                DRIVETRAIN_CONSTANTS.getDrivetrainModPositions()
            ));
        }

        public static final double DRIVETRAIN_HEADING_CONTROLLER_KP = 0.07;
        public static final double DRIVETRAIN_HEADING_CONTROLLER_KI = 0.006;
        public static final double DRIVETRAIN_HEADING_CONTROLLER_KD = 0.001;

        public static final SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(
            0.60757, 7.6216,0.71241
        );
        
        public static final KinematicLimits DRIVETRAIN_UNCAPPED = new KinematicLimits(
            DRIVETRAIN_CONSTANTS.getFreeSpeedMetersPerSecond(),
            Double.POSITIVE_INFINITY,
            720.0
        );
        public static final KinematicLimits DRIVETRAIN_SMOOTHED = new KinematicLimits(
            DRIVETRAIN_CONSTANTS.getFreeSpeedMetersPerSecond(),
            10.0,
            500.0
        );
        public static final KinematicLimits DRIVETRAIN_LIMITED = new KinematicLimits(
            2.0,
            7.0,
            300.0
        );


        public static final SwerveModuleConstants MOD0 = new SwerveModuleConstants();
        static {
            MOD0.setModuleNumber(0);

            MOD0.setDriveMotorPort(Ports.CANID.DRIVETRAIN_FRONT_LEFT_DRIVE);
            MOD0.setAngleMotorPort(Ports.CANID.DRIVETRAIN_BACK_LEFT_STEER);

            MOD0.setAngleOffsetDegreesCCW(0);
            MOD0.setInvertAngleOutput(false);
            MOD0.setInvertAngleSensorPhase(false);
            MOD0.setOnCanivore(false);
        }

        public static final SwerveModuleConstants MOD1 = new SwerveModuleConstants();
        static {
            MOD1.setModuleNumber(1);
            
            MOD1.setDriveMotorPort(Ports.CANID.DRIVETRAIN_FRONT_RIGHT_DRIVE);
            MOD1.setAngleMotorPort(Ports.CANID.DRIVETRAIN_FRONT_RIGHT_STEER);

            MOD1.setAngleOffsetDegreesCCW(0);
            MOD1.setInvertAngleOutput(false);
            MOD1.setInvertAngleSensorPhase(false);
            MOD1.setOnCanivore(false);
        }

        public static final SwerveModuleConstants MOD2 = new SwerveModuleConstants();
        static {
            MOD2.setModuleNumber(2);
            
            MOD2.setDriveMotorPort(Ports.CANID.DRIVETRAIN_BACK_LEFT_DRIVE);
            MOD2.setAngleMotorPort(Ports.CANID.DRIVETRAIN_BACK_LEFT_STEER);

            MOD2.setAngleOffsetDegreesCCW(0);
            MOD2.setInvertAngleOutput(false);
            MOD2.setInvertAngleSensorPhase(false);
            MOD2.setOnCanivore(false);
        }

        public static final SwerveModuleConstants MOD3 = new SwerveModuleConstants();
        static {
            MOD3.setModuleNumber(3);
            
            MOD3.setDriveMotorPort(Ports.CANID.DRIVETRAIN_BACK_RIGHT_DRIVE);
            MOD3.setAngleMotorPort(Ports.CANID.DRIVETRAIN_BACK_RIGHT_STEER);

            MOD3.setAngleOffsetDegreesCCW(0);
            MOD3.setInvertAngleOutput(false);
            MOD3.setInvertAngleSensorPhase(false);
            MOD3.setOnCanivore(false);
        }

    }

    public static final class ControllerConstants {
        public static boolean INVERT_X = false;
        public static boolean INVERT_Y = false;
        public static boolean INVERT_R = false;
        public static double DEADBAND = 0.05;
    }
}
