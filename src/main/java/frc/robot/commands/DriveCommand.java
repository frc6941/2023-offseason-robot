package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.Swerve;

public class DriveCommand extends CommandBase {
    private ControlBoard controlBoard;
    private final Swerve swerve;

    public DriveCommand(Swerve swerve) {
        this.swerve = swerve;
        controlBoard = ControlBoard.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(controlBoard.getSwerveTranslation(), controlBoard.getSwerveRotation(), true, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
