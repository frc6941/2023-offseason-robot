package frc.robot.controlboard;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.controlboard.CustomXboxController.Axis;
import frc.robot.controlboard.CustomXboxController.Button;
import frc.robot.controlboard.CustomXboxController.Side;
import frc.robot.controlboard.SwerveCardinal.SWERVE_CARDINAL;
import org.frcteam6328.utils.TunableNumber;

public class ControlBoard {
    public final double swerveDeadband = Constants.ControllerConstants.DEADBAND;

    private static ControlBoard instance = null;

    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }
        return instance;
    }

    private final CustomXboxController driver;
    private final CustomButtonBoard operator;

    private final TunableNumber controllerCurveStrength = new TunableNumber("Controller Curve Strength", 0.7);

    private ControlBoard() {
        driver = new CustomXboxController(Ports.Controller.DRIVER);
        operator = new CustomButtonBoard(Ports.Controller.OPERATOR);
    }

    public CustomXboxController getDriverController() {
        return driver;
    }

    public CustomButtonBoard getOperatorController() {
        return operator;
    }

    public void setDriverRumble(double power, double interval) {
        driver.setRumble(power, interval);
    }

    public void updateRumble(double time) {
        driver.updateRumble(time);
    }


    ////////// DRIVER //////////
    /**
     * Get normalized swerve translation with respect to set deadband.
     * @return Translation2d required swerve translational velocity, normalized
     */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = cubicCurved(driver.getAxis(Side.LEFT, Axis.Y), controllerCurveStrength.get());
        double strafeAxis = cubicCurved(driver.getAxis(Side.LEFT, Axis.X), controllerCurveStrength.get());


        forwardAxis = Constants.ControllerConstants.INVERT_Y ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.ControllerConstants.INVERT_X ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < swerveDeadband) {
            return new Translation2d();
        }

        return tAxes;
    }

    private double cubicCurved(double value, double strength) {
        return (1 - strength) * value + strength * Math.pow(value, 3);
    }

    /** 
     * Get normalized swerve rotation with respect to set deadband.
     * @return Translation2d required swerve rotational velocity, normalized
     */
    public double getSwerveRotation() {
        double rotAxis = cubicCurved(driver.getAxis(Side.RIGHT, Axis.X), controllerCurveStrength.get());
        rotAxis = Constants.ControllerConstants.INVERT_R ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < swerveDeadband) {
            return 0.0;
        }

        return (rotAxis - (Math.signum(rotAxis) * swerveDeadband)) / (1 - swerveDeadband);
    }
    
    public Trigger zeroGyro() {
        return new Trigger(() -> driver.getController().getStartButtonPressed());
    }

    /** 
     * Get the targeted snap rotation direction for swerve to turn to.
     * @return SWERVE_CARDINAL the target snap direction
     */
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

    public Trigger getIntake() {
        return driver.buttonPressed(Button.RB);
    }

    // Locks wheels in X formation
    public Trigger getSwerveBrake() {
        return new Trigger(() -> driver.getButton(Button.R_JOYSTICK));
    }

    public boolean getRobotOriented() {
        return driver.getTrigger(Side.LEFT) > 0.3;
    }

    public Trigger getAutoShoot() {
        return new Trigger(() -> driver.getTrigger(Side.RIGHT) > 0.3);
    }

    ////////// OPERATOR //////////
    public Trigger getToggleClimbMode() {
        return operator.buttonPressed(frc.robot.controlboard.CustomButtonBoard.Button.UM);
    }

    public Trigger getClimbConfirmation() {
        return operator.buttonPressed(frc.robot.controlboard.CustomButtonBoard.Button.MM);
    }

    public Trigger getHookForward() {
        return operator.buttonPressed(frc.robot.controlboard.CustomButtonBoard.Button.UL);
    }

    public Trigger getHookReverse() {
        return operator.buttonPressed(frc.robot.controlboard.CustomButtonBoard.Button.UR);
    }

    public Trigger getPusherForward() {
        return operator.buttonPressed(frc.robot.controlboard.CustomButtonBoard.Button.ML);
    }

    public Trigger getPusherReverse() {
        return operator.buttonPressed(frc.robot.controlboard.CustomButtonBoard.Button.MR);
    }

    public Trigger getPointShot() {
        return operator.buttonPressed(frc.robot.controlboard.CustomButtonBoard.Button.LL);
    }

    public Trigger getFenderShot() {
        return operator.buttonPressed(frc.robot.controlboard.CustomButtonBoard.Button.LM);        
    }

    public Trigger getForceReverse() {
        return operator.buttonPressed(frc.robot.controlboard.CustomButtonBoard.Button.LR);        
    }

    public Trigger tempQueueCorrectBall() {
        return new Trigger(() -> driver.getController().getXButtonPressed());
    }
    public Trigger tempQueueWrongBall() {
        return new Trigger(() -> driver.getController().getYButtonPressed());
    }
    public Trigger tempReverse() {
        return new Trigger(() -> driver.getController().getBButtonPressed());
    }
}
