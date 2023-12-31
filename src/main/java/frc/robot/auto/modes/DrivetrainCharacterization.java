package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.basics.AutoMode;
import frc.robot.commands.CharacterizationDriveCommand;
import frc.robot.subsystems.Swerve;

public class DrivetrainCharacterization implements AutoMode {
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
        characterize = new CharacterizationDriveCommand(swerve, 1.0, 0.5, 5.0);
    }

    public Command getAutoCommand() {
        return characterize;
    }

    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}