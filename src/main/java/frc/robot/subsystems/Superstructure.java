package frc.robot.subsystems;

import org.frcteam6941.looper.Updatable;

public class Superstructure implements Updatable {
    private final ColorSensor colorSensor = ColorSensor.getInstance();
    private final Indexer indexer = Indexer.getInstance();

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
        if(colorSensor.seesNewBall()) {
            System.out.println("Queue Ball!");
            indexer.queueBall(colorSensor.hasCorrectColor());
        }
    }

    @Override
    public void update(double time, double dt) {
        queueBalls();
    }
}
