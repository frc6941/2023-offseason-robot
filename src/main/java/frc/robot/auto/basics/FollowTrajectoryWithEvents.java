package frc.robot.auto.basics;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

import java.util.Map;

public class FollowTrajectoryWithEvents extends FollowPathWithEvents {
    Swerve swerve;
    public FollowTrajectoryWithEvents(Swerve swerve, PathPlannerTrajectory trajectory, boolean lockAngle, boolean requiredOnTarget, Map<String, Command> eventMap) {
        super(new FollowTrajectory(swerve, trajectory, lockAngle, requiredOnTarget), trajectory.getMarkers(), eventMap);
        this.swerve = swerve;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopMovement();
    }
}
