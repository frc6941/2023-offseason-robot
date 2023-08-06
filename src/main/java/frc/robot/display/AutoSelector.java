package frc.robot.display;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.basics.AutoMode;
import frc.robot.auto.modes.DrivetrainCharacterization;
import frc.robot.auto.modes.FlywheelCharacterization;
import frc.robot.auto.modes.TestAuto;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import org.frcteam6328.utils.Alert;
import org.frcteam6328.utils.Alert.AlertType;

import java.util.List;
import java.util.Optional;

import static frc.robot.auto.modes.AutoModeBuilder.buildFullAutoMode;

public class AutoSelector {
    private AutoMode autoMode;
    private final SendableChooser<AutoMode> modeChooser = new SendableChooser<>();
    private final Alert autoAlert = new Alert("Alerts/Program", "No valid auto is chosen.", AlertType.WARNING);

    private final List<AutoMode> availableModes = List.of(
            buildFullAutoMode("D 3S", "D 3S"),
            buildFullAutoMode("D 5S", "D 5S"),
            buildFullAutoMode("D 5S + 1O", "D 5S + 1O"),
            buildFullAutoMode("D 6S", "D 6S"),
            buildFullAutoMode("D 6S + 1O", "D 6S + 1O"),
            buildFullAutoMode("A 2S + 1O", "A 2S + 1O"),
            buildFullAutoMode("A 2S + 2O", "A 2S + 2O"),
            buildFullAutoMode("B 1S", "B 1S"),
            buildFullAutoMode("B 1S + 1O", "B 1S + 1O"),
            buildFullAutoMode("B 1SN", "B 1SN"),
            new TestAuto(),
            new DrivetrainCharacterization(Swerve.getInstance()),
            new FlywheelCharacterization(Shooter.getInstance())
    );

    private static AutoSelector instance;

    public static AutoSelector getInstance() {
        if (instance == null) {
            instance = new AutoSelector();
        }
        return instance;
    }

    private AutoSelector() {
        availableModes.forEach((AutoMode mode) -> {
            modeChooser.addOption(mode.getAutoName(), mode);
        });
        autoAlert.set(true);
    }

    public void updateModeCreator() {
        AutoMode tempMode = modeChooser.getSelected();
        if (tempMode != null) {
            if (autoMode != tempMode) {
                resetStartingPosition(tempMode.getStartingPose());
            }
            autoMode = modeChooser.getSelected();
            autoAlert.set(autoMode.shouldWarn());
        }
    }

    public void resetStartingPosition(Pose2d pose) {
        Swerve.getInstance().resetPose(pose);
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
