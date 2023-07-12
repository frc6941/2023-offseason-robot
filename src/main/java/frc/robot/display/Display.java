package frc.robot.display;

import java.util.Optional;

import org.frcteam6941.looper.Updatable;

import frc.robot.Constants;
import frc.robot.auto.modes.AutoMode;
import frc.robot.subsystems.Swerve;

public class Display implements Updatable {
    FieldView fieldView = new FieldView();
    AutoSelector selector = AutoSelector.getInstance();
    ShootingParametersTable table = ShootingParametersTable.getInstance();
    OperatorDashboard operator = OperatorDashboard.getInstance();

    Swerve swerve = Swerve.getInstance();

    
    private static Display instance;

    private Display() {

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
    public void telemetry() {
        fieldView.update(swerve.getLocalizer().getLatestPose(), swerve.getModuleStates(), Constants.SwerveConstants.DRIVETRAIN_CONSTANTS);
        selector.updateModeCreator();
        table.update();
    }

    @Override
    public void stop() {
    }
}
