package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Trigger;

public class ForceReverseCommand extends FunctionalCommand {
    public ForceReverseCommand(Indexer indexer, Trigger trigger) {
        super(
                () -> {},
                () -> {
                    indexer.setWantForceReverse(true);
                    trigger.reverse(false);

                },
                (interrupted) -> {
                    indexer.setWantForceReverse(false);
                    trigger.lock();
                },
                () -> false
        );
        addRequirements(indexer, trigger);
    }
}
