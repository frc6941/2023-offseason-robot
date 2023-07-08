package frc.robot;

import frc.robot.commands.AutoFenderShootCommand;
import frc.robot.commands.DefaultIndicatorCommand;
import frc.robot.subsystems.*;
import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.modes.AutoMode;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.display.Display;
import frc.robot.display.ShootingParametersTable;

public class RobotContainer {
    private final UpdateManager updateManager;

    private final Swerve swerve = Swerve.getInstance();
    private final ColorSensor colorSensor = ColorSensor.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Trigger trigger = Trigger.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();

    private final Superstructure superstructure = Superstructure.getInstance();

    private final Aim aim = Aim.getInstance();
    private final ShootingParametersTable shootingParametersTable = ShootingParametersTable.getInstance();

    private final Indicator indicator = Indicator.getInstance();
    private final Display display = Display.getInstance();

    private final ControlBoard controlBoard = ControlBoard.getInstance();

    public RobotContainer() {
        updateManager = new UpdateManager(
                swerve,
                colorSensor,
                indexer,
                trigger,
                shooter,
                hood,
                superstructure,
                aim,
                indicator,
                display
        );
        updateManager.registerAll();

        bindDefaultCommands();
        bindControlBoard();
    }

    private void bindDefaultCommands() {
        indicator.setDefaultCommand(
                new DefaultIndicatorCommand(indicator, indexer)
        );
    }

    private void bindControlBoard() {
        swerve.setDefaultCommand(
                new DriveTeleopCommand(
                        swerve,
                        controlBoard::getSwerveTranslation,
                        controlBoard::getSwerveRotation,
                        controlBoard::getRobotOriented,
                        () -> controlBoard.getSwerveSnapRotation().degrees
                )
        );
        controlBoard.zeroGyro().whenActive(
                new InstantCommand(() -> {
                    Translation2d translation = swerve.getLocalizer().getLatestPose().getTranslation();
                    swerve.resetPose(
                            new Pose2d(
                                    translation,
                                    new Rotation2d()
                            )
                    );
                })
        );
        controlBoard.getAutoShoot().whileActiveContinuous(
                new AutoShootCommand(swerve, indexer, trigger, shooter, hood, aim, indicator, shootingParametersTable, () -> false)
        );


        controlBoard.getFenderShot().whileActiveOnce(
                new AutoFenderShootCommand(indexer, trigger, shooter, hood, indicator, shootingParametersTable)
        );
    }

    public UpdateManager getUpdateManager() {
        return updateManager;
    }

    public Command getAutonomousCommand() {
        return Display
                .getInstance()
                .getSelectedAutoMode()
                .orElseGet(AutoMode::new)
                .getAutoCommand();
    }
}
