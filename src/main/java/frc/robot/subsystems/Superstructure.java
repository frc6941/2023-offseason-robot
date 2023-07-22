package frc.robot.subsystems;

import lombok.Getter;
import lombok.Setter;
import org.frcteam6941.looper.Updatable;

public class Superstructure implements Updatable {
    private final ColorSensor colorSensor = ColorSensor.getInstance();
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
        if (!intaker.seesNewBall()) return;

        if(indexer.isFull()) {
            intaker.setForceOff(true);
        } else {
            intaker.setForceOff(false);
            indexer.queueBall(overrideColorSensor || colorSensor.hasCorrectColor());
        }
    }

    @Override
    public void update(double time, double dt) {
        queueBalls();
    }
}
