package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CharacterizeMotorCommand;
import frc.robot.subsystems.Shooter;

public class FlywheelCharacterization extends AutoMode {
    @Override
    public String getAutoName() {
        return "Flywheel Characterization";
    }

    @Override
    public boolean shouldWarn() {
        return true;
    }

    Command characterize;

    public FlywheelCharacterization(Shooter shooter) {
        characterize = new CharacterizeMotorCommand(shooter, 0.0, 0.2, 12.0);
    }

    public Command getAutoCommand() {
        return characterize;
    }

    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}