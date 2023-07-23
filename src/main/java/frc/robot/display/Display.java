package frc.robot.display;

import frc.robot.Constants;
import frc.robot.auto.basics.AutoMode;
import frc.robot.controlboard.ControlBoard;
import frc.robot.states.AimingParameters;
import frc.robot.subsystems.Aim;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import org.frcteam6941.looper.Updatable;

import java.util.Optional;

public class Display implements Updatable {
    FieldView fieldView = new FieldView();
    AutoSelector selector = AutoSelector.getInstance();
    ShootingParametersTable table = ShootingParametersTable.getInstance();
    OperatorDashboard dashboard = OperatorDashboard.getInstance();
    Swerve swerve = Swerve.getInstance();
    Aim aim = Aim.getInstance();
    Shooter shooter = Shooter.getInstance();
    Hood hood = Hood.getInstance();


    private static Display instance;

    private Display() {
        OperatorDashboard.getInstance();
    }

    public static Display getInstance() {
        if (instance == null) {
            instance = new Display();
        }
        return instance;
    }

    public Optional<AutoMode> getSelectedAutoMode() {
        return selector.getAutoMode();
    }

    @Override
    public void update(double time, double dt) {
        ControlBoard.getInstance().updateRumble(time);
    }

    @Override
    public void telemetry() {
        fieldView.update(swerve.getLocalizer().getCoarseFieldPose(0.0), swerve.getLocalizer().getLatestPose(), swerve.getModuleStates(), Constants.SwerveConstants.DRIVETRAIN_CONSTANTS);
        selector.updateModeCreator();
        table.update();

        AimingParameters aimingParameters = aim.getAimingParameters(-1).orElse(aim.getDefaultAimingParameters());
        dashboard.getCurrentDistance().setDouble(aimingParameters.getVehicleToTarget().getTranslation().getNorm());
        dashboard.getCurrentRpm().setDouble(shooter.getShooterRPM());
        dashboard.getCurrentHoodAngle().setDouble(hood.getHoodAngle());
    }
}
