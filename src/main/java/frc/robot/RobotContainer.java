package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.auto.basics.AutoMode;
import frc.robot.auto.basics.EmptyAutoMode;
import frc.robot.commands.*;
import frc.robot.controlboard.ControlBoard;
import frc.robot.display.Display;
import frc.robot.display.ShootingParametersTable;
import frc.robot.subsystems.*;
import org.frcteam6941.looper.UpdateManager;

import java.util.concurrent.locks.Lock;

public class RobotContainer {
    private final UpdateManager updateManager;

    private final Swerve swerve = Swerve.getInstance();

    private final Intaker intaker = Intaker.getInstance();
    private final ColorSensorRio colorSensorRio = ColorSensorRio.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Trigger trigger = Trigger.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();

    private final Climber climber = Climber.getInstance();

    private final Superstructure superstructure = Superstructure.getInstance();

    private final Limelight limelight = Limelight.getInstance();
    private final Aim aim = Aim.getInstance();
    private final ShootingParametersTable shootingParametersTable = ShootingParametersTable.getInstance();

    private final Indicator indicator = Indicator.getInstance();
    private final Display display = Display.getInstance();

    private final ControlBoard controlBoard = ControlBoard.getInstance();

    public RobotContainer() {
        updateManager = new UpdateManager(
                swerve,
                intaker,
                indexer,
                colorSensorRio,
                trigger,
                shooter,
                hood,
                superstructure,
                limelight,
                aim,
                indicator,
                climber,
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
                        () -> !controlBoard.getRobotOriented(),
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
                new AutoShootCommand(
                        swerve, indexer, trigger, shooter,
                        hood, aim, indicator, shootingParametersTable,
                        () -> false, true
                ),
                false
        );


        controlBoard.getFenderShot().whileActiveContinuous(
                new AutoFenderShootCommand(indexer, trigger, shooter, hood, indicator, shootingParametersTable)
        );


        controlBoard.getIntake().whileActiveContinuous(new AutoIntakeCommand(intaker)).whenInactive(
              new SequentialCommandGroup(
                      new WaitCommand(0.5),
                      new InstantCommand(intaker::stopRolling)
              )
        );

        controlBoard.getPusherForward().whenActive(
                new ClimbSetHookCommand(climber, 1000.0)
        );

        controlBoard.getPusherReverse().whenActive(
                new ClimbSetHookCommand(climber, 0.0)
        );

        controlBoard.getToggleClimbMode().toggleWhenActive(
                new AutoClimbCommand(climber, indicator, () -> controlBoard.getClimbConfirmation().getAsBoolean())
        );

        controlBoard.getForceReverse().whileActiveContinuous(
                new FunctionalCommand(
                        () -> {
                            indexer.setWantForceReverse(true);
                            trigger.reverse(false);
                            intaker.roll(
                                    -Constants.IntakerConstants.ROLLING_VOLTAGE.get(),
                                    -Constants.IntakerConstants.HOPPER_VOLTAGE.get()
                            );
                        },
                        () -> {},
                        (interrupted) -> {
                            indexer.setWantForceReverse(false);
                            trigger.lock();
                        },
                        () -> { return false; }
                )
        ).whenInactive(
                new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new InstantCommand(intaker::stopRolling)
                )
        );

        controlBoard.getResetColorSensor().whileActiveContinuous(
                new ResetColorSensorCommand(colorSensorRio, indicator)
        );

        controlBoard.getHold().whenActive(
                new InstantCommand(() -> indexer.setWantHold(true))
        ).whenInactive(
                new InstantCommand(() -> indexer.setWantHold(false))
        );

        new edu.wpi.first.wpilibj2.command.button.Trigger(() -> indexer.isFull() && DriverStation.isTeleopEnabled()).whileActiveContinuous(
                new InstantCommand(
                        () -> controlBoard.setDriverRumble(1.0, 0.5)
                )
        ).whenInactive(
                new InstantCommand(
                        () -> controlBoard.setDriverRumble(0.0, 0.0)
                )
        );

        new edu.wpi.first.wpilibj2.command.button.Trigger(() -> RobotController.getUserButton() && DriverStation.isDisabled()).toggleWhenActive(
                new LockClimber(climber, indicator)
        );
    }

    public UpdateManager getUpdateManager() {
        return updateManager;
    }

    public Command getAutonomousCommand() {
        return Display
                .getInstance()
                .getSelectedAutoMode()
                .orElseGet(EmptyAutoMode::new)
                .getAutoCommand();
    }
}
