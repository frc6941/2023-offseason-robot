package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Trigger;

public class ForceEjectCommand extends FunctionalCommand {
    public ForceEjectCommand(Indexer indexer, Trigger trigger) {
        super(
                () -> {},
                () -> {
                    indexer.setWantForceEject(true);
                    trigger.reverse(false);
                },
                (interrupted) -> {
                    indexer.setWantForceEject(false);
                    indexer.setFastEject(true);
                    trigger.lock();
                },
                () -> false
        );
        addRequirements(indexer, trigger);
    }
}
