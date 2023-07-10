package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intaker;

public class RollCommand extends CommandBase {
    private final Intaker intaker;

    public RollCommand(Intaker intaker) {
        this.intaker = intaker;
        addRequirements(this.intaker);
    }

    @Override
    public void execute() {
        intaker.roll(
                Constants.IntakerConstants.ROLLING_VOLTAGE.get(),
                Constants.IntakerConstants.HOPPER_VOLTAGE.get()
        );
    }


    @Override
    public void end(boolean interrupted) {
        intaker.stopRolling();
    }
}
