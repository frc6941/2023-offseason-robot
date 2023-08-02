package frc.robot.auto.basics;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FullAutoBuilder {
    private final Consumer<Pose2d> resetPose;
    private final Map<String, Command> eventMap;
    private final Swerve swerve;

    public FullAutoBuilder(
            Swerve swerve,
            Consumer<Pose2d> resetPose,
            Map<String, Command> eventMap) {
        this.resetPose = resetPose;
        this.eventMap = eventMap;
        this.swerve = swerve;
    }

    private static CommandBase wrappedEventCommand(Command eventCommand) {
        return new FunctionalCommand(
                eventCommand::initialize,
                eventCommand::execute,
                eventCommand::end,
                eventCommand::isFinished,
                eventCommand.getRequirements().toArray(Subsystem[]::new));
    }

    private CommandBase getStopEventCommands(PathPlannerTrajectory.StopEvent stopEvent) {
        List<CommandBase> commands = new ArrayList<>();

        int startIndex = stopEvent.executionBehavior == PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL_DEADLINE ? 1 : 0;
        for (int i = startIndex; i < stopEvent.names.size(); i++) {
            String name = stopEvent.names.get(i);
            if (eventMap.containsKey(name)) {
                commands.add(wrappedEventCommand(eventMap.get(name)));
            }
        }

        switch (stopEvent.executionBehavior) {
            case SEQUENTIAL:
                return new SequentialCommandGroup(commands.toArray(CommandBase[]::new));
            case PARALLEL:
                return new ParallelCommandGroup(commands.toArray(CommandBase[]::new));
            case PARALLEL_DEADLINE:
                Command deadline =
                        eventMap.containsKey(stopEvent.names.get(0))
                                ? wrappedEventCommand(eventMap.get(stopEvent.names.get(0)))
                                : new InstantCommand();
                return new ParallelDeadlineGroup(deadline, commands.toArray(CommandBase[]::new));
            default:
                throw new IllegalArgumentException(
                        "Invalid stop event execution behavior: " + stopEvent.executionBehavior);
        }
    }

    private CommandBase stopEventGroup(PathPlannerTrajectory.StopEvent stopEvent) {
        if (stopEvent.names.isEmpty()) {
            return new WaitCommand(stopEvent.waitTime);
        }

        CommandBase eventCommands = getStopEventCommands(stopEvent);

        switch (stopEvent.waitBehavior) {
            case BEFORE:
                return new SequentialCommandGroup(new WaitCommand(stopEvent.waitTime), eventCommands);
            case AFTER:
                return new SequentialCommandGroup(eventCommands, new WaitCommand(stopEvent.waitTime));
            case DEADLINE:
                return new ParallelDeadlineGroup(new WaitCommand(stopEvent.waitTime), eventCommands);
            case MINIMUM:
                return new ParallelCommandGroup(new WaitCommand(stopEvent.waitTime), eventCommands);
            case NONE:
            default:
                return eventCommands;
        }
    }

    private CommandBase followPathWithEvents(PathPlannerTrajectory trajectory) {
        return new FollowTrajectoryWithEvents(swerve, trajectory, true, false, eventMap);
    }

    private CommandBase resetPose(PathPlannerTrajectory trajectory) {
        return new InstantCommand(
            () -> {
            PathPlannerTrajectory.PathPlannerState initialState = trajectory.getInitialState();
            resetPose.accept(
                    new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
        });
    }


    public CommandBase fullAuto(PathPlannerTrajectory trajectory) {
        return fullAuto(new ArrayList<>(List.of(trajectory)));
    }

    public CommandBase fullAuto(List<PathPlannerTrajectory> pathGroup) {
        List<CommandBase> commands = new ArrayList<>();

        commands.add(resetPose(pathGroup.get(0)));

        for (PathPlannerTrajectory traj : pathGroup) {
            commands.add(stopEventGroup(traj.getStartStopEvent()));
            commands.add(followPathWithEvents(traj));
        }

        commands.add(stopEventGroup(pathGroup.get(pathGroup.size() - 1).getEndStopEvent()));

        return new SequentialCommandGroup(commands.toArray(CommandBase[]::new));

    }
}
