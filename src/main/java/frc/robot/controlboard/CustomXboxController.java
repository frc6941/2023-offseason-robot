package frc.robot.controlboard;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CustomXboxController {
    private final XboxController controller;

    private double rumblePower = 0.0;
    private double rumbleInterval = 0.0;

    public enum Side {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    public enum Button {
        A(1), B(2), X(3), Y(4), LB(5), RB(6), BACK(7), START(8), L_JOYSTICK(9), R_JOYSTICK(10);

        public final int id;

        Button(int id) {
            this.id = id;
        }
    }

    public CustomXboxController(int port) {
        controller = new XboxController(port);
    }

    double getAxis(Side side, Axis axis) {
        boolean left = side == Side.LEFT;
        boolean y = axis == Axis.Y;
        return controller.getRawAxis((left ? 0 : 4) + (y ? 1 : 0));
    }

    public double getTrigger(Side side) {
        return controller.getRawAxis(side == Side.LEFT ? 2 : 3);
    }

    public boolean getTriggerBoolean(Side side) {
        return controller.getRawAxis(side == Side.LEFT ? 2 : 3) > 0.5;
    }

    public boolean getButton(Button button) {
        return controller.getRawButton(button.id);
    }

    public Trigger buttonPressed(Button button) {
        return new Trigger(() -> this.getButton(button));
    }

    public void setRumble(double power, double interval) {
        rumblePower = power;
        rumbleInterval = interval;
    }

    public void updateRumble(double time) {
        if (rumbleInterval == 0) {
            controller.setRumble(RumbleType.kRightRumble, rumblePower);
            controller.setRumble(RumbleType.kLeftRumble, rumblePower);
        } else if (Math.floor(time / rumbleInterval) % 2 == 0) {
            controller.setRumble(RumbleType.kRightRumble, rumblePower);
            controller.setRumble(RumbleType.kLeftRumble, rumblePower);
        } else {
            controller.setRumble(RumbleType.kRightRumble, 0.0);
            controller.setRumble(RumbleType.kLeftRumble,0.0);
        }
    }

    public XboxController getController() {
        return controller;
    }
}
