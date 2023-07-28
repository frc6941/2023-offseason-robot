package frc.robot.subsystems;

import frc.robot.controlboard.ControlBoard;
import lombok.Getter;
import lombok.Setter;
import org.frcteam6941.looper.Updatable;

public class Superstructure implements Updatable {
    private final ColorSensorRio colorSensor = ColorSensorRio.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Intaker intaker = Intaker.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();


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
                indexer.queueBall(!ControlBoard.getInstance().getManualWrongBall().getAsBoolean());
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
    }
}
