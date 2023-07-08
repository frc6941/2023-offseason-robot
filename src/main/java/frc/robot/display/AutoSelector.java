package frc.robot.display;

import java.util.List;
import java.util.Optional;

import org.frcteam6328.utils.Alert;
import org.frcteam6328.utils.Alert.AlertType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.modes.AutoMode;
import frc.robot.auto.modes.DrivetrainCharacterization;
import frc.robot.auto.modes.FlywheelCharacterization;
import frc.robot.auto.modes.TestAuto;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoSelector {
    private AutoMode autoMode;
    private final SendableChooser<AutoMode> modeChooser = new SendableChooser<>();
    private final Alert autoAlert = new Alert("Alerts/Program", "No valid auto is chosen.", AlertType.WARNING);

    private final List<AutoMode> availableModes = List.of(
        new TestAuto(),
        new DrivetrainCharacterization(Swerve.getInstance()),
        new FlywheelCharacterization(Shooter.getInstance())
    );

    public AutoSelector() {
        availableModes.forEach((AutoMode mode) -> {
            modeChooser.addOption(mode.getAutoName(), mode);
        });
        autoAlert.set(true);
    }

    public void updateModeCreator() {
        AutoMode tempMode = modeChooser.getSelected();
        if (tempMode != null) {
            if(autoMode != tempMode) {
                resetStartingPosition(tempMode.getStartingPose());
            }
            autoMode = modeChooser.getSelected();
            autoAlert.set(autoMode.shouldWarn());
        }
    }

    public void resetStartingPosition(Pose2d pose) {
        Swerve.getInstance().getLocalizer().reset(pose);
    }


    public void reset() {
        autoMode = null;
    }

    public Optional<AutoMode> getAutoMode() {
        return Optional.ofNullable(autoMode);
    }

    public SendableChooser<AutoMode> getSendableChooser() {
        return modeChooser;
    }


}
