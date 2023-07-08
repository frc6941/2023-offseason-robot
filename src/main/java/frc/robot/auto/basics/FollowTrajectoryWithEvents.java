package frc.robot.auto.basics;

import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class FollowTrajectoryWithEvents extends FollowPathWithEvents {
    public FollowTrajectoryWithEvents(Swerve swerve, PathPlannerTrajectory trajectory, boolean lockAngle, boolean requiredOnTarget, Map<String, Command> eventMap) {
        super(new FollowTrajectory(swerve, trajectory, lockAngle, requiredOnTarget), trajectory.getMarkers(), eventMap);
    }
}
