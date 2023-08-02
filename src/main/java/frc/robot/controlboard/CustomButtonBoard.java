package frc.robot.controlboard;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CustomButtonBoard extends GenericHID {
    public CustomButtonBoard(int port) {
        super(port);
    }

    public enum Button {
        LL(1), LM(2), LR(3), ML(4), MM(5), MR(6), UM(7), UL(11), UR(12);

        public final int id;

        Button(int id) {
            this.id = id;
        }
    }

    public Trigger button(Button button) {
        return new Trigger(() -> this.getRawButton(button.id));
    }

    public Trigger buttonPressed(Button button) {
        return new Trigger(() -> this.getRawButtonPressed(button.id));
    }

    public Trigger buttonReleased(Button button) {
        return new Trigger(() -> this.getRawButtonReleased(button.id));
    }

}