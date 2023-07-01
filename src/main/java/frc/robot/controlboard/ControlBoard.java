package frc.robot.controlboard;

import org.frcteam6328.utils.TunableNumber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.controlboard.CustomXboxController.Axis;
import frc.robot.controlboard.CustomXboxController.Button;
import frc.robot.controlboard.CustomXboxController.Side;
import frc.robot.controlboard.SwerveCardinal.SWERVE_CARDINAL;

public class ControlBoard {
    public final double kSwerveDeadband = Constants.ControllerConstants.DEADBAND;

    private static ControlBoard instance = null;

    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }
        return instance;
    }

    private final CustomXboxController driver;
    private final CustomXboxController operator;

    private final TunableNumber controllerCurveStrength = new TunableNumber("Controller Curve Strength", 0.7);

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
    

    
    /** 
     * Get normalized swerve translation with respect to setted deadbands.
     * @return Translation2d required swerve translational velocity, normalized
     */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = cubicCurved(driver.getAxis(Side.LEFT, Axis.Y), controllerCurveStrength.get());
        double strafeAxis = cubicCurved(driver.getAxis(Side.LEFT, Axis.X), controllerCurveStrength.get());


        forwardAxis = Constants.ControllerConstants.INVERT_Y ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.ControllerConstants.INVERT_X ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            return tAxes;
        }
    }

    private double cubicCurved(double value, double strength) {
        return (1 - strength) * value + strength * Math.pow(value, 3);
    }

    /** 
     * Get normalized swerve rotation with respect to setted deadbands.
     * @return Translation2d required swerve rotational velocity, normalized
     */
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

    // Locks wheels in X formation
    public Trigger getSwerveBrake() {
        return new Trigger(() -> driver.getButton(Button.R_JOYSTICK));
    }

    
}
