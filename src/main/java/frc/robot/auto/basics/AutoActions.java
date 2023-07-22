package frc.robot.auto.basics;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.Map;

public class AutoActions {
    private final static Swerve swerve = Swerve.getInstance();

    public static Command followTrajectoryWithEvents(PathPlannerTrajectory trajectory, boolean lockAngle) {
        return new FollowTrajectoryWithEvents(swerve, trajectory, lockAngle, true, eventMap);
    }

    public static Command followTrajectory(PathPlannerTrajectory trajectory, boolean lockAngle) {
        return new FollowTrajectory(swerve, trajectory, lockAngle, true);
    }

    public static Command resetOdometry(Pose2d startingPose) {
        return new InstantCommand(() -> swerve.getLocalizer().reset(startingPose));
    }

    private static Map<String, Command> eventMap = new HashMap<>();

    static {
        eventMap.put("test", new InstantCommand(() -> System.out.println("Command Mapping Test.")));
    }
}
