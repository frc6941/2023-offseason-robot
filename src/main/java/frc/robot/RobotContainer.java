package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.controlboard.CustomXboxController;
import frc.robot.subsystems.Swerve;
import org.frcteam6941.looper.UpdateManager;

public class RobotContainer {
    private final UpdateManager updateManager;

    private final Swerve swerve = Swerve.getInstance();

    CustomXboxController driverController = new CustomXboxController(Ports.CONTROLLER.DRIVER);
    CustomXboxController operatorController = new CustomXboxController(Ports.CONTROLLER.OPERATOR);

    public RobotContainer() {
        updateManager = new UpdateManager(
                swerve
        );
        bindControlBoard();
        swerve.setDefaultCommand(
                new DriveCommand(swerve)
        );
    }

    private void bindControlBoard() {
    }

    public UpdateManager getUpdateManager() {
        return updateManager;
    }
}
