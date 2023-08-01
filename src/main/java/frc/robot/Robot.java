// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        if (Constants.IS_REAL) {
            robotContainer.getUpdateManager().startEnableLoop(Constants.LOOPER_DT);
        } else {
            robotContainer.getUpdateManager().startSimulateLoop(Constants.LOOPER_DT);
        }

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();

        robotContainer.getUpdateManager().invokeStop();
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

        Command autoCommand = robotContainer.getAutonomousCommand();
        CommandScheduler.getInstance().schedule(autoCommand);

        robotContainer.getUpdateManager().invokeStart();
        Superstructure.getInstance().setOverrideColorSensor(true);
        Indexer.getInstance().reset();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();
        robotContainer.getUpdateManager().invokeStart();

        Shooter.getInstance().idle();
        Intaker.getInstance().stopRolling();
        Superstructure.getInstance().setOverrideColorSensor(false);
        Indexer.getInstance().setFastEject(true);
        Climber.getInstance().setHookAngle(0.0);
        Climber.getInstance().setPusherAngle(0.0);
        Indexer.getInstance().reset();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }
}
