// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frcteam6941.looper.UpdateManager;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    Swerve swerve = Swerve.getInstance();
    ControlBoard controlBoard = ControlBoard.getInstance();

    private UpdateManager updateManager;
    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        this.updateManager = new UpdateManager(
            swerve
        );
        this.updateManager.startEnableLoop(Constants.LOOPER_DT);

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        this.updateManager.stopEnableLoop();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected.
     */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        this.updateManager.startEnableLoop(Constants.LOOPER_DT);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        swerve.drive(controlBoard.getSwerveTranslation(), controlBoard.getSwerveRotation(), true, false);
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }
}
