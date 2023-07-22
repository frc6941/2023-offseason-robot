package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoMode {
    public String getAutoName() {
        return "Default";
    }

    ;

    public boolean shouldWarn() {
        return true;
    }

    ;

    public Pose2d getStartingPose() {
        return new Pose2d();
    }

    ;

    public Command getAutoCommand() {
        return new InstantCommand();
    }

    ;

    public AutoMode() {

    }
}

