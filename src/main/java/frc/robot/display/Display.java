package frc.robot.display;

import org.frcteam6941.looper.Updatable;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Display implements Updatable {
    FieldView fieldView = new FieldView();

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

    @Override
    public void telemetry() {
        fieldView.update(swerve.getLocalizer().getLatestPose(), swerve.getModuleStates(), Constants.SwerveConstants.DRIVETRAIN_CONSTANTS);
    }
}
