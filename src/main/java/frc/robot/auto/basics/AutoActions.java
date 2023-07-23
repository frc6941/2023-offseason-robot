package frc.robot.auto.basics;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.commands.IntakeDeployCommand;
import frc.robot.auto.commands.IntakeRetractCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.display.ShootingParametersTable;
import frc.robot.states.ShootingParameters;
import frc.robot.subsystems.*;
import lombok.Getter;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AutoActions {
    private final static Swerve swerve = Swerve.getInstance();
    private final static Indexer indexer = Indexer.getInstance();
    private final static Intaker intaker = Intaker.getInstance();
    private final static Trigger trigger = Trigger.getInstance();
    private final static Shooter shooter = Shooter.getInstance();
    private final static Hood hood = Hood.getInstance();
    private final static Limelight limelight = Limelight.getInstance();
    private final static Indicator indicator = Indicator.getInstance();
    private final static Aim aim = Aim.getInstance();
    private final static ShootingParametersTable parameters = ShootingParametersTable.getInstance();

    @Getter
    private static Map<String, Command> eventMap = new HashMap<>();

    static {
        eventMap.put("deploy", new IntakeDeployCommand(intaker));
        eventMap.put("retract", new IntakeRetractCommand(intaker));
        eventMap.put("shoot", new AutoShootCommand(swerve, indexer, trigger, shooter, hood, aim, indicator, parameters, () -> false));
        eventMap.put("test", new InstantCommand(() -> System.out.println("Command Mapping Test.")));
    }


    private final static FullAutoBuilder autoBuilder = new FullAutoBuilder(
            swerve,
            pose2d -> swerve.getLocalizer().reset(pose2d),
            eventMap
    );

    public static PathPlannerTrajectory getTrajectory(String name, PathConstraints constraints) {
        return PathPlanner.loadPath(name, constraints);
    }

    public static List<PathPlannerTrajectory> getTrajectoryGroup(String name, PathConstraints constraints) {
        return PathPlanner.loadPathGroup(name, constraints);
    }

    public static Command followTrajectoryWithEvents(PathPlannerTrajectory trajectory, boolean lockAngle) {
        return new FollowTrajectoryWithEvents(swerve, trajectory, lockAngle, true, eventMap);
    }

    public static Command followTrajectory(PathPlannerTrajectory trajectory, boolean lockAngle) {
        return new FollowTrajectory(swerve, trajectory, lockAngle, true);
    }

    public static Command resetOdometry(Pose2d startingPose) {
        return new InstantCommand(() -> swerve.getLocalizer().reset(startingPose));
    }

    public static Command fullAuto(PathPlannerTrajectory trajectory) {
        return autoBuilder.fullAuto(trajectory);
    }

    public static Command fullAuto(List<PathPlannerTrajectory> trajectories) {
        return autoBuilder.fullAuto(trajectories);
    }
}
