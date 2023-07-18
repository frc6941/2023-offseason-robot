package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.modes.AutoMode;
import frc.robot.commands.*;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.CustomXboxController;
import frc.robot.display.Display;
import frc.robot.display.ShootingParametersTable;
import frc.robot.states.ShootingParameters;
import frc.robot.subsystems.*;
import org.frcteam6941.looper.UpdateManager;

public class RobotContainer {
    private final UpdateManager updateManager;

    private final Swerve swerve = Swerve.getInstance();
    private final ColorSensorRio colorSensor = ColorSensorRio.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Trigger trigger = Trigger.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();

    private final Superstructure superstructure = Superstructure.getInstance();

    private final Aim aim = Aim.getInstance();
    private final ShootingParametersTable shootingParametersTable = ShootingParametersTable.getInstance();

    private final Indicator indicator = Indicator.getInstance();
    private final Display display = Display.getInstance();

    private final Intaker intaker = Intaker.getInstance();
    private final ControlBoard controlBoard = ControlBoard.getInstance();

    public RobotContainer() {
        updateManager = new UpdateManager(
                swerve,
                intaker,
                colorSensor,
                indexer,
                trigger,
                shooter,
//                hood,
                superstructure,
//                aim,
//                indicator,
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
//                        () -> controlBoard.getSwerveSnapRotation().degrees
                        () -> null
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
                new AutoShootCommand(
                        swerve, indexer, trigger, shooter,
                        hood, aim, indicator, shootingParametersTable,
                        () -> false
                ),
                false
        );


        controlBoard.getFenderShot().whileActiveContinuous(
                new AutoFenderShootCommand(indexer, trigger, shooter, hood, indicator, shootingParametersTable)
        );


        controlBoard.getIntake().whileActiveContinuous(new AutoIntakeCommand(intaker));

        new edu.wpi.first.wpilibj2.command.button.Trigger(
                () -> controlBoard.getDriverController().getButton(CustomXboxController.Button.X)
        ).whileActiveContinuous(
                new ShootCommand(
                        indexer, trigger, shooter, hood,
                        shootingParametersTable::getCustomShotParameters
                )
        );

        new edu.wpi.first.wpilibj2.command.button.Trigger(indexer::isFull).whileActiveContinuous(
                new InstantCommand(
                        () -> controlBoard.setDriverRumble(1.0, 0.5)
                )
        ).whenInactive(
                new InstantCommand(
                        () -> controlBoard.setDriverRumble(0.0, 0.0)
                )
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
