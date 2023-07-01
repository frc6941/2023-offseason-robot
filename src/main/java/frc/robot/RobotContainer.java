package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.CustomXboxController;
import frc.robot.display.Display;
import frc.robot.subsystems.Swerve;
import org.frcteam6941.looper.UpdateManager;

public class RobotContainer {
    private final UpdateManager updateManager;

    private final Swerve swerve = Swerve.getInstance();
    private final Display display = Display.getInstance();

    private final ControlBoard controlBoard = ControlBoard.getInstance();

    public RobotContainer() {
        updateManager = new UpdateManager(
            swerve,
            display
        );
        updateManager.registerAll();

        swerve.setDefaultCommand(
            new DriveCommand(
                swerve,
                () -> controlBoard.getSwerveTranslation(),
                () -> controlBoard.getSwerveRotation(),
                () -> true,
                () -> controlBoard.getSwerveSnapRotation().degrees
            )
        );

        bindControlBoard();
    }

    private void bindControlBoard() {
    }

    public UpdateManager getUpdateManager() {
        return updateManager;
    }
}
