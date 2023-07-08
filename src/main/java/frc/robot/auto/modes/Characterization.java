package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveLinearCharacterizationCommand;
import frc.robot.subsystems.Swerve;

public class Characterization extends AutoMode {
    @Override
    public String getAutoName() {
        return "Characterization";
    }

    @Override
    public boolean shouldWarn() {
        return true;
    }

    DriveLinearCharacterizationCommand characterize;

    public Characterization(Swerve swerve) {
        characterize = new DriveLinearCharacterizationCommand(swerve, 0, 0.2);
    }

    public Command getAutoCommand() {
        return characterize;
    }

    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}