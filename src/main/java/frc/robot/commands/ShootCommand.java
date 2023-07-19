package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.display.OperatorDashboard;
import frc.robot.states.Lights;
import frc.robot.subsystems.*;
import org.frcteam6328.utils.TunableNumber;
import org.frcteam6941.utils.AngleNormalization;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.states.AimingParameters;
import frc.robot.states.ShootingParameters;
import frc.robot.display.ShootingParametersTable;

public class ShootCommand extends CommandBase {
    private final Indexer indexer;
    private final Trigger trigger;
    private final Shooter shooter;
    private final Hood hood;


    private final Supplier<ShootingParameters> targetParameter;

    private boolean isSpunUp = false;
    private boolean isHoodUp = false;

    public ShootCommand(Indexer indexer, Trigger trigger, Shooter shooter, Hood hood, Supplier<ShootingParameters> targetParameter) {
        this.indexer = indexer;
        this.trigger = trigger;
        this.shooter = shooter;
        this.hood = hood;
        this.targetParameter = targetParameter;
        addRequirements(trigger, shooter, hood);
    }

    private void judgeStatus() {
        isSpunUp = Util.epsilonEquals(
                shooter.getShooterRPM(),
                targetParameter.get().getVelocityRpm(),
                Constants.JudgeConstants.FLYWHEEL_RPM_TOLERANCE
        );

        isHoodUp = hood.isCalibrated() && Util.epsilonEquals(
                hood.getHoodAngle(),
                targetParameter.get().getBackboardAngleDegree(),
                Constants.JudgeConstants.BACKBOARD_ANGLE_TOLERANCE
        );
    }

    private void setMechanisms() {
        hood.setHoodAngle(targetParameter.get().getBackboardAngleDegree());
        shooter.setShooterRPM(targetParameter.get().getVelocityRpm());

        if (isSpunUp && isHoodUp) {
            trigger.feed(false);
            indexer.setWantFeed(true);
        } else {
            trigger.feed();
            indexer.setWantFeed(false);
        }
    }

    private void clearTelemetry() {
        OperatorDashboard feedback = OperatorDashboard.getInstance();
        feedback.getReady().setBoolean(false);
        feedback.getLockOn().setBoolean(false);
        feedback.getSpunUp().setBoolean(false);
    }


    @Override
    public void initialize() {
        shooter.turnOff();
        trigger.lock();
        indexer.setWantFeed(false);
    }

    @Override
    public void execute() {
        judgeStatus();
        setMechanisms();
    }

    @Override
    public void end(boolean isInterrupted) {
        shooter.turnOff();
        trigger.lock();
        indexer.setWantFeed(false);
        clearTelemetry();
    }
}
