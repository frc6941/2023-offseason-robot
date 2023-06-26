package frc.robot.controlboard;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.controlboard.CustomXboxController.Axis;
import frc.robot.controlboard.CustomXboxController.Button;
import frc.robot.controlboard.CustomXboxController.Side;
import frc.robot.controlboard.SwerveCardinal.SWERVE_CARDINAL;

public class ControlBoard {
    public final double kSwerveDeadband = Constants.ControllerConstants.DEADBAND;

    private final int kDpadUp = 0;
    private final int kDpadRight = 90;
    private final int kDpadDown = 180;
    private final int kDpadLeft = 270;

    private static ControlBoard instance = null;

    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }
        return instance;
    }

    private final CustomXboxController driver;
    private final CustomXboxController operator;

    private ControlBoard() {
        driver = new CustomXboxController(Ports.CONTROLLER.DRIVER);
        operator = new CustomXboxController(Ports.CONTROLLER.OPERATOR);
    }

    public CustomXboxController getDriverController() {
        return driver;
    }

    public CustomXboxController getOperatorController() {
        return operator;
    }

    public void setDriverRumble(double power, double interval) {
        driver.setRumble(power, interval);
    }

    public void setOperatorRumble(double power, double interval) {
        operator.setRumble(power, interval);
    }

    /* DRIVER METHODS */
    /**
     * Get swerve translation in normalized terms.
     * @return Swerve translation velocity.
     */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = driver.getAxis(Side.LEFT, Axis.Y);
        double strafeAxis = driver.getAxis(Side.LEFT, Axis.X);

        forwardAxis = Constants.ControllerConstants.INVERT_Y ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.ControllerConstants.INVERT_X ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            return tAxes;
        }
    }

    public double getSwerveRotation() {
        double rotAxis = driver.getAxis(Side.RIGHT, Axis.X) * 2.0;
        rotAxis = Constants.ControllerConstants.INVERT_R ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < kSwerveDeadband) {
            return 0.0;
        } else {
            return (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);
        }
    }

    public boolean zeroGyro() {
        return driver.getController().getStartButtonPressed();
    }

    public SWERVE_CARDINAL getSwerveSnapRotation() {
        if (driver.getButton(Button.A)) {
            return SWERVE_CARDINAL.BACKWARDS;
        } else if (driver.getButton(Button.X)) {
            return SWERVE_CARDINAL.RIGHT;
        } else if (driver.getButton(Button.B)) {
            return SWERVE_CARDINAL.LEFT;
        } else if (driver.getButton(Button.Y)) {
            return SWERVE_CARDINAL.FORWARDS;
        } else {
            return SWERVE_CARDINAL.NONE;
        }

    }

    public boolean getSwitchEject() {
        return driver.getController().getPOV() == kDpadDown;
    }

    // Locks wheels in X formation
    public boolean getSwerveBrake() {
        return driver.getButton(Button.R_JOYSTICK);
    }

    // Robot oriented switch
    public boolean getSwitchRobotOrientedDrive() {
        return driver.getController().getLeftStickButtonPressed();
    }

    // Start intake
    public boolean getIntake() {
        return driver.getButton(Button.LB);
    }

    public boolean getShoot() {
        return driver.getButton(Button.RB);
    }

    public boolean getSpit() {
        return driver.getController().getPOV() == kDpadDown;
    }

    public boolean getDecreaseShotAdjustment() {
        return driver.getController().getPOV() == kDpadRight;
    }

    public boolean getIncreaseShotAdjustment() {
        return driver.getController().getPOV() == kDpadLeft;
    }

    public boolean getSwitchCompressorForceEnable() {
        return driver.getController().getBackButtonPressed();
    }

    // Climber Controls
    public boolean getEnterClimbMode() {
        return operator.getButton(Button.LB) && operator.getButton(Button.RB) && operator.getTriggerBoolean(Side.LEFT)
                && operator.getTriggerBoolean(Side.RIGHT);
        // return operator.getButton(Button.LB) && operator.getButton(Button.RB);
    }

    public boolean getExitClimbMode() {
        return operator.getButton(Button.BACK) && operator.getButton(Button.START);
    }

    public boolean getToggleOpenLoopClimbMode() {
        return operator.getController().getLeftStickButtonPressed();
    }

    public boolean getClimberRetract() {
        return operator.getController().getPOV() == kDpadDown;
    }

    public boolean getClimberExtend() {
        return operator.getController().getPOV() == kDpadUp;
    }

    public boolean getClimberDown() {
        return operator.getController().getPOV() == kDpadDown;
    }

    public boolean getClimberUp() {
        return operator.getController().getPOV() == kDpadUp;
    }

    public boolean getClimberHootOut() {
        return operator.getController().getPOV() == kDpadRight;
    }

    public boolean getClimberHootIn() {
        return operator.getController().getPOV() == kDpadLeft;
    }

    public boolean getTraversalClimb() {
        return operator.getController().getYButtonPressed();
    }

    public boolean getHighBarClimb() {
        return operator.getController().getBButtonPressed();
    }

    public boolean getClimbAutoConfirmation() {
        return operator.getButton(Button.A);
    }

    public boolean getClimbAutoAbort() {
        return operator.getButton(Button.X);
    }
}
