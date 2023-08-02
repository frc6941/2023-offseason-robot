package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.team254.lib.util.MovingAverage;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.controlboard.ControlBoard;
import lombok.Getter;
import lombok.Setter;
import org.checkerframework.checker.units.qual.A;
import org.checkerframework.checker.units.qual.C;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.pathplanning.universal.Path;

public class Superstructure implements Updatable {
    private final ColorSensorRio colorsensor = ColorSensorRio.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Intaker intaker = Intaker.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();
    private final ControlBoard controlBoard = ControlBoard.getInstance();

    private Timer delayedJudge = new Timer();

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

    private void queueBalls() {
        if (intaker.seesNewBall()) {
            if(DriverStation.isTeleopEnabled()) {
                new SequentialCommandGroup(
                        new InstantCommand(() -> controlBoard.setDriverRumble(1.0, 0.0)),
                        new InstantCommand(() -> {
                            if(ControlBoard.getInstance().getRobotOriented()){
                                controlBoard.setOperatorRumble(1.0, 0.0);
                            }
                        }),
                        new WaitCommand(0.4),
                        new InstantCommand(() -> controlBoard.setDriverRumble(0.0, 0.0)),
                        new InstantCommand(() -> controlBoard.setOperatorRumble(0.0, 0.0))
                ).schedule();
            }

            if (indexer.isFull()) {
                intaker.setForceOff(true);
            } else {
                intaker.setForceOff(false);
                delayedJudge.reset();
                delayedJudge.start();
            }
        }

        if(delayedJudge.get() > 0.05) {
            System.out.println(colorsensor.hasCorrectColor());
            indexer.queueBall(colorsensor.hasCorrectColor() || overrideColorSensor);
            delayedJudge.reset();
            delayedJudge.stop();
        }
    }

    public void inClimb() {
        shooter.turnOff();
        indexer.turnOff();
        intaker.intakerBrake();
        hood.setHoodMinimum();
    }

    @Override
    public void update(double time, double dt) {
        queueBalls();
    }
}
