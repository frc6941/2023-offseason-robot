package frc.robot.auto.basics;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.PrepMechanismsCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ForceEjectCommand;
import frc.robot.commands.ForceReverseCommand;
import frc.robot.display.ShootingParametersTable;
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
        eventMap.put("deploy", deploy());
        eventMap.put("retract", retract());
        eventMap.put("intake", intake());
        eventMap.put("reverse", reverse());
        eventMap.put("eject", eject());
        eventMap.put("shoot", shoot());
        eventMap.put("test", new InstantCommand(() -> System.out.println("Command Mapping Test.")));
    }


    private final static FullAutoBuilder autoBuilder = new FullAutoBuilder(
            swerve,
            swerve::resetPose,
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
        return new InstantCommand(() -> swerve.resetPose(startingPose));
    }

    public static Command deploy() {
        return new InstantCommand(intaker::deploy).alongWith(
                new InstantCommand(() -> intaker.roll(
                        Constants.IntakerConstants.ROLLING_VOLTAGE.get(),
                        Constants.IntakerConstants.HOPPER_VOLTAGE.get()
                ))
        );
    }

    public static Command retract() {
        return new InstantCommand(intaker::retract).alongWith(new InstantCommand(intaker::stopRolling));
    }

    public static Command intake() {
        return new InstantCommand(() -> intaker.roll(
                Constants.IntakerConstants.ROLLING_VOLTAGE.get(),
                Constants.IntakerConstants.HOPPER_VOLTAGE.get()
        ));
    }

    public static Command reverse() {
        return new ForceReverseCommand(indexer, trigger).alongWith(
                new InstantCommand(() -> intaker.roll(
                        -Constants.IntakerConstants.ROLLING_VOLTAGE.get(),
                        -Constants.IntakerConstants.HOPPER_VOLTAGE.get()
                ))
        );
    }

    public static Command eject() {
        return new ForceEjectCommand(indexer, trigger);
    }

    public static Command shoot() {
        return new AutoShootCommand(swerve, indexer, trigger, shooter, hood, aim, indicator, parameters, () -> false);
    }

    public static Command safeShoot() {
        return new ParallelDeadlineGroup(
                waitForFeeding().andThen(waitFor(1.0)),
                shoot()
        );
    }

    public static Command prepShooting(Pose2d preaimPose) {
        return new PrepMechanismsCommand(shooter, hood, preaimPose);
    }

    public static Command waitFor(double seconds) {
        return new WaitCommand(seconds);
    }

    public static Command waitForFeeding() {
        return new WaitUntilCommand(() -> indexer.getState() == Indexer.State.FEEDING);
    }

    public static Command print(String message) {
        return new PrintCommand(message);
    }

    public static Command fullAuto(PathPlannerTrajectory trajectory) {
        return autoBuilder.fullAuto(trajectory);
    }

    public static Command fullAuto(List<PathPlannerTrajectory> trajectories) {
        return autoBuilder.fullAuto(trajectories);
    }
}
