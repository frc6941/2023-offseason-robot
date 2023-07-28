package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controlboard.ControlBoard;
import lombok.Getter;
import lombok.Setter;
import org.checkerframework.checker.units.qual.A;
import org.frcteam6941.looper.Updatable;

public class Superstructure implements Updatable {
    private final ColorSensorRio colorSensor = ColorSensorRio.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Intaker intaker = Intaker.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();

    private final AnalogInput simpleColorSensor = new AnalogInput(3);

    private boolean hasCorrectBall() {
        DriverStation.Alliance alliance = DriverStation.getAlliance();
        return (simpleColorSensor.getAverageVoltage() < 0.8 && alliance == DriverStation.Alliance.Red)
                || (simpleColorSensor.getAverageVoltage() > 0.8 && alliance == DriverStation.Alliance.Blue);
    }


    @Getter
    @Setter
    private boolean overrideColorSensor = true;

    private static Superstructure instance;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    private Superstructure() {
    }

    private void queueBalls() {
        if (intaker.seesNewBall()) {
//            indexer.queueBall(overrideColorSensor || colorSensor.hasCorrectColor());

            if (indexer.isFull()) {
                intaker.setForceOff(true);
            } else {
                intaker.setForceOff(false);
                indexer.queueBall(true);
            }
        }
    }

    private void inClimb() {
        shooter.turnOff();
        hood.setHoodMinimum();
    }

    @Override
    public void update(double time, double dt) {
        queueBalls();
        SmartDashboard.putBoolean("Cor B", hasCorrectBall());
        SmartDashboard.putNumber("Cor B V", simpleColorSensor.getAverageVoltage());
    }
}
