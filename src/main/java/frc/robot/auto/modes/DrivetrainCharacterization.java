package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CharacterizationDriveCommand;
import frc.robot.subsystems.Swerve;

public class DrivetrainCharacterization extends AutoMode {
    @Override
    public String getAutoName() {
        return "Drivetrain Characterization";
    }

    @Override
    public boolean shouldWarn() {
        return true;
    }

    CharacterizationDriveCommand characterize;

    public DrivetrainCharacterization(Swerve swerve) {
        characterize = new CharacterizationDriveCommand(swerve, 0, 0.5, 12.0);
    }

    public Command getAutoCommand() {
        return characterize;
    }

    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}