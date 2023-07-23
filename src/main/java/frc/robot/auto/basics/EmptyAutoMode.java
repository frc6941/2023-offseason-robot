package frc.robot.auto.basics;

import edu.wpi.first.wpilibj2.command.Command;

public class EmptyAutoMode implements AutoMode{
    @Override
    public String getAutoName() {
        return null;
    }

    @Override
    public boolean shouldWarn() {
        return true;
    }
}
