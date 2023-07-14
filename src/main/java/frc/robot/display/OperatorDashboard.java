package frc.robot.display;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import lombok.Getter;

@Getter
public class OperatorDashboard {
    private final ShuffleboardTab operatorTab;

    // Superstructure Status
    private final NetworkTableEntry ready;
    private final NetworkTableEntry lockOn;
    private final NetworkTableEntry spunUp;
    private final NetworkTableEntry hasTarget;

    // Ball Path status
    private final NetworkTableEntry ballFull;
    private final NetworkTableEntry ballPathEjecting;
    private final NetworkTableEntry ballPathFeeding;
    private final NetworkTableEntry ballPathIndexing;

    // Aiming status
    private final NetworkTableEntry currentDistance;
    private final NetworkTableEntry currentRpm;
    private final NetworkTableEntry currentHoodAngle;
    private final NetworkTableEntry currentAimDelta;

    private static OperatorDashboard instance;

    public static OperatorDashboard getInstance() {
        if (instance == null) {
            instance = new OperatorDashboard();
        }
        return instance;
    }


    private OperatorDashboard() {
        operatorTab = Shuffleboard.getTab("Operator");

        ready = operatorTab
                .add("Ready", false)
                .withSize(3, 3)
                .withPosition(1, 0)
                .getEntry();
        hasTarget = operatorTab
                .add("Has Target", false)
                .withSize(1, 1)
                .withPosition(4, 0)
                .getEntry();
        spunUp = operatorTab
                .add("Spun Up", false)
                .withSize(1, 1)
                .withPosition(4, 1)
                .getEntry();
        lockOn = operatorTab
                .add("Lock On", false)
                .withSize(1, 1)
                .withPosition(4, 2)
                .getEntry();

        ballPathIndexing = operatorTab
                .add("BallPath Indexing", false)
                .withSize(1, 1)
                .withPosition(7, 0)
                .getEntry();
        ballPathFeeding = operatorTab
                .add("BallPath Feeding", false)
                .withSize(1, 1)
                .withPosition(7, 1)
                .getEntry();
        ballPathEjecting = operatorTab
                .add("BallPath Ejecting", false)
                .withSize(1, 1)
                .withPosition(7, 2)
                .getEntry();
        ballFull = operatorTab
                .add("BallPath Full", false)
                .withSize(3, 3)
                .withPosition(8, 0)
                .getEntry();


        currentDistance = operatorTab
                .add("Current Distance", -1.0)
                .withSize(1, 1)
                .withPosition(4, 4)
                .getEntry();
        currentRpm = operatorTab
                .add("Current Rpm", -1.0)
                .withSize(1, 1)
                .withPosition(5, 4)
                .getEntry();
        currentHoodAngle = operatorTab
                .add("Current Hood Angle", -1.0)
                .withSize(1, 1)
                .withPosition(6, 4)
                .getEntry();
        currentAimDelta = operatorTab
                .add("Current Aim Delta", -1.0)
                .withSize(1, 1)
                .withPosition(7, 4)
                .getEntry();

        operatorTab
                .add("Auto Selector", AutoSelector.getInstance().getSendableChooser())
                .withSize(3,2)
                .withPosition(1, 3);
//        operatorTab
//                .add("Match Time", false)
//                .withWidget("Match Time")
//                .withSize(2,1)
//                .withPosition(5, 3);
    }
}
