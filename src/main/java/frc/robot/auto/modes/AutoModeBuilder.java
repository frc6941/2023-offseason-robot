package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.basics.AutoMode;

import java.util.List;

import static frc.robot.auto.basics.AutoActions.fullAuto;

public class AutoModeBuilder {
    public static AutoMode buildFullAutoMode(String autoName, String pathFileName, boolean isValid) {
        return new AutoMode() {

            private final List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup(pathFileName, 3.7, 4.0, false);

            public Command getAutoCommand() {
                return fullAuto(trajectories);
            }

            public Pose2d getStartingPose() {
                return trajectories.get(0).getInitialHolonomicPose();
            }
            @Override
            public String getAutoName() {
                return autoName;
            }

            @Override
            public boolean shouldWarn() {
                return isValid;
            }
        };
    }

    public static AutoMode buildFullAutoMode(String autoName, String pathFileName) {
        return buildFullAutoMode(autoName, pathFileName, true);
    }
}
