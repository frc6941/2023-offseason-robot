package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.basics.AutoActions;
import frc.robot.auto.basics.AutoMode;

import java.util.List;

import static frc.robot.auto.basics.AutoActions.*;

public class TestAuto implements AutoMode {
    @Override
    public String getAutoName() {
        return "Test Auto";
    }

    @Override
    public boolean shouldWarn() {
        return true;
    }

    private final List<PathPlannerTrajectory> testTrajectory;

    public TestAuto() {
        testTrajectory = PathPlanner.loadPathGroup("Test Trajectory", 3.5, 5.0, false);
    }

    public Command getAutoCommand() {
        return fullAuto(testTrajectory);
    }

    public Pose2d getStartingPose() {
        return testTrajectory.get(0).getInitialHolonomicPose();
    }
}