package frc.robot.auto.basics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public interface AutoMode {
    String getAutoName();

    boolean shouldWarn();

    default Pose2d getStartingPose() {
        return new Pose2d();
    }

    default Command getAutoCommand() {
        return new InstantCommand();
    }
}

