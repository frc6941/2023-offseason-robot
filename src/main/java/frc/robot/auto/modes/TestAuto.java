package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.auto.basics.AutoActions.*;

public class TestAuto extends AutoMode {
    @Override
    public String getAutoName() {
        return "Test Auto";
    }

    @Override
    public boolean shouldWarn() {
        return true;
    }
    PathPlannerTrajectory testTrajectory;
    
    public TestAuto() {
        testTrajectory = PathPlanner.loadPath("Test Trajectory", 3.5, 5.0, false);
    }

    public Command getAutoCommand() {
        return resetOdometry(testTrajectory.getInitialHolonomicPose())
                .andThen(followTrajectoryWithEvents(testTrajectory, true));
    }

    public Pose2d getStartingPose() {
        return testTrajectory.getInitialHolonomicPose();
    }
}