package frc.robot.commands;

import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.display.ShootingParametersTable;
import frc.robot.states.Lights;
import frc.robot.states.ShootingParameters;
import frc.robot.subsystems.*;

public class AutoFenderShootCommand extends CommandBase {
    private final Indexer indexer;
    private final Trigger trigger;
    private final Shooter shooter;
    private final Hood hood;
    private final Indicator indicator;
    private final ShootingParametersTable parametersTable;

    private ShootingParameters parameters;
    private boolean isSpunUp = false;
    private boolean isHoodUp = false;

    public AutoFenderShootCommand(Indexer indexer, Trigger trigger, Shooter shooter, Hood hood, Indicator indicator, ShootingParametersTable parametersTable) {
        this.indexer = indexer;
        this.trigger = trigger;
        this.shooter = shooter;
        this.hood = hood;
        this.indicator = indicator;
        this.parametersTable = parametersTable;
        parameters = parametersTable.getFenderShotParameters();
        addRequirements(trigger, shooter, hood, indicator);
    }

    private void judgeStatus() {
        isSpunUp = Util.epsilonEquals(
                shooter.getShooterRPM(),
                parameters.getVelocityRpm(),
                Constants.JudgeConstants.FLYWHEEL_RPM_TOLERANCE
        );

        isHoodUp = hood.isCalibrated() && Util.epsilonEquals(
                hood.getHoodAngle(),
                parameters.getBackboardAngleDegree(),
                Constants.JudgeConstants.BACKBOARD_ANGLE_TOLERANCE
        );
    }

    private void setMechanisms() {
        hood.setHoodAngle(parameters.getBackboardAngleDegree());
        shooter.setShooterRPM(parameters.getVelocityRpm());

        if (isSpunUp && isHoodUp) {
            trigger.feed(false);
            indexer.setWantFeed(true);
        } else {
            trigger.lock();
            indexer.setWantFeed(false);
        }
    }

    private void setIndicator() {
        indicator.setIndicatorState(Lights.FENDER);
    }

    @Override
    public void initialize() {
        shooter.turnOff();
        indicator.clearIndicator();
        trigger.lock();
        indexer.setWantFeed(false);
    }

    @Override
    public void execute() {
        parameters = parametersTable.getFenderShotParameters();
        judgeStatus();
        setMechanisms();
        setIndicator();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.idle();
        trigger.lock();
        indexer.setWantFeed(false);
        hood.setHoodMinimum();
    }
}
