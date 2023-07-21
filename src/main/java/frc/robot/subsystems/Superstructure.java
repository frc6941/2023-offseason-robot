package frc.robot.subsystems;

import frc.robot.controlboard.ControlBoard;
import lombok.Getter;
import lombok.Setter;
import org.frcteam6941.drivers.BeamBreak;
import org.frcteam6941.looper.Updatable;

public class Superstructure implements Updatable {
    private final ColorSensorRio colorSensor = ColorSensorRio.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Intaker intaker = Intaker.getInstance();

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
        if(intaker.seesNewBall()) {
//            indexer.queueBall(overrideColorSensor || colorSensor.hasCorrectColor());
            indexer.queueBall(ControlBoard.getInstance().getDriverController().getController().getPOV() != 90);
        }
    }

    @Override
    public void update(double time, double dt) {
        queueBalls();
    }
}
