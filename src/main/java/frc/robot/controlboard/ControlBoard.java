package frc.robot.controlboard;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
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
    private final CustomXboxController operator;

    private final TunableNumber controllerCurveStrength = new TunableNumber("Driver Curve Strength", 0.3);
    private final TunableNumber operatorCurveStrength = new TunableNumber("Operator Curve Strength", 0.9);

    private ControlBoard() {
        driver = new CustomXboxController(Ports.Controller.DRIVER);
        operator = new CustomXboxController(Ports.Controller.OPERATOR);
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

    public void updateRumble(double time) {
        driver.updateRumble(time);
        operator.updateRumble(time);
    }


    ////////// DRIVER //////////
    private double cubicCurved(double value, double strength) {
        return (1 - strength) * value + strength * Math.pow(value, 3);
    }

    /**
     * Get normalized swerve translation with respect to set deadband.
     *
     * @return Translation2d required swerve translational velocity, normalized
     */
    public Translation2d getDriverSwerveTranslation() {
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


    /**
     * Get normalized swerve rotation with respect to set deadband.
     *
     * @return Translation2d required swerve rotational velocity, normalized
     */
    public double getDriverSwerveRotation() {
        double rotAxis = cubicCurved(driver.getAxis(Side.RIGHT, Axis.X), controllerCurveStrength.get());
        rotAxis = Constants.ControllerConstants.INVERT_R ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < swerveDeadband) {
            return 0.0;
        }

        return (rotAxis - (Math.signum(rotAxis) * swerveDeadband)) / (1 - swerveDeadband);
    }

    public Translation2d getOperatorSwerveTranslation() {
        double forwardAxis = cubicCurved(operator.getAxis(Side.LEFT, Axis.Y), operatorCurveStrength.get());
        double strafeAxis = cubicCurved(operator.getAxis(Side.LEFT, Axis.X), operatorCurveStrength.get());


        forwardAxis = Constants.ControllerConstants.INVERT_Y ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.ControllerConstants.INVERT_X ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < swerveDeadband) {
            return new Translation2d();
        }

        return tAxes;
    }


    /**
     * Get normalized swerve rotation with respect to set deadband.
     *
     * @return Translation2d required swerve rotational velocity, normalized
     */
    public double getOperatorSwerveRotation() {
        double rotAxis = cubicCurved(operator.getAxis(Side.RIGHT, Axis.X), operatorCurveStrength.get());
        rotAxis = Constants.ControllerConstants.INVERT_R ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < swerveDeadband) {
            return 0.0;
        }

        return (rotAxis - (Math.signum(rotAxis) * swerveDeadband)) / (1 - swerveDeadband);
    }

    public Trigger zeroGyro() {
        return new Trigger(() -> driver.getController().getStartButtonPressed());
    }

    public Trigger zeroGyroOpposite() {
        return new Trigger(() -> driver.getController().getBackButtonPressed());
    }

    /**
     * Get the targeted snap rotation direction for swerve to turn to.
     *
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
        return new Trigger(() ->
            driver.buttonPressed(Button.LB).get() && !getRobotOriented()
                || (operator.buttonPressed(Button.LB).get() && getRobotOriented())
        );
    }

    public Trigger getHold() {
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

    public Trigger tuningRPMUp() {
        return new Trigger(() -> driver.getController().getPOV() == 0);
    }

    public Trigger tuningRPMDown() {
        return new Trigger(() -> driver.getController().getPOV() == 180);
    }

    public Trigger tuningHoodUp() {
        return new Trigger(() -> driver.getController().getPOV() == 90);
    }

    public Trigger tuningHoodDown() {
        return new Trigger(() -> driver.getController().getPOV() == 270);
    }

    public Trigger getResetIntake() {
        return new Trigger(() -> driver.getController().getBackButton());
    }


    ////////// OPERATOR //////////
    public Trigger getToggleClimbMode() {
        return new Trigger(() -> operator.getTrigger(Side.LEFT) > 0.5 && operator.getTrigger(Side.RIGHT) > 0.5);
    }

    public Trigger getClimbConfirmation() {
        return operator.buttonPressed(Button.A);
        
    }

    public Trigger getBallCounterUp() {
        return new Trigger(() -> operator.getController().getPOV() == 90);
    }

    public Trigger getBallCounterDown() {
        return new Trigger(() -> operator.getController().getPOV() == 270);
    }

    public Trigger getFenderShot() {
        return new Trigger(() -> operator.getButton(Button.X));
    }

    public Trigger getForceReverse() {
        return new Trigger(() -> operator.getButton(Button.B));
    }

    public Trigger getOverrideColorSensor() {
        return new Trigger(() -> operator.getButton(Button.START));
    }

    public Trigger getResetColorSensor() {
        return new Trigger(
                () -> operator.getButton(Button.L_JOYSTICK) &&
                RobotState.isDisabled()
        );
    }

    public Trigger tuningGetDistanceUp() {
        return new Trigger(
                () -> operator.getController().getPOV() == 0
        );
    }

    public Trigger tuningGetDistanceDown() {
        return new Trigger(
                () -> operator.getController().getPOV() == 180
        );
    }

    public Trigger tuningGetDriveToDistance() {
        return new Trigger(() -> operator.getButton(Button.Y));
    }
}
