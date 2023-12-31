package frc.robot.commands;

import com.team254.lib.util.PolynomialRegression;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.swerve.SwerveModuleBase;

import java.util.ArrayList;

public class CharacterizationDriveCommand extends CommandBase {
    private Swerve drivetrain;
    private double startVoltage;
    private double deltaVoltage;
    private final double maxVoltage;
    private final double prepTime = 3.0;

    private final Timer prepTimer = new Timer();
    private final Timer timer = new Timer();

    private final ArrayList<Double> yVoltages = new ArrayList<>();
    private final ArrayList<Double> xVelocities = new ArrayList<>();
    private final ArrayList<Double> xFalconVelocities = new ArrayList<>();

    private double travelTicks = 0;

    public CharacterizationDriveCommand(Swerve drivetrain, double startVoltage, double deltaVoltage, double maxVoltage) {
        this.drivetrain = drivetrain;
        this.startVoltage = startVoltage;
        this.deltaVoltage = deltaVoltage;
        this.maxVoltage = maxVoltage;
    }

    private final Runnable r = () -> {
        if (prepTimer.get() < prepTime) {
            timer.stop();
            SwerveModuleState individualState = new SwerveModuleState(
                    0.0, new Rotation2d());
            drivetrain.setModuleStates(new SwerveModuleState[]{
                    individualState, individualState, individualState, individualState
            }, true, true);
            return;
        }

        timer.start();
        double targetVoltage = startVoltage + deltaVoltage * timer.get();

        SwerveModuleState individualState = new SwerveModuleState(
                targetVoltage / 12.0, new Rotation2d()
        );
        drivetrain.setModuleStates(new SwerveModuleState[]{
                individualState,
                individualState,
                individualState,
                individualState
        }, true, true);
        yVoltages.add(targetVoltage);

        SwerveModuleState[] moduleStates = drivetrain.getModuleStates();
        double averageVelocity = 0.0;
        double averageFalconVelocity = 0.0;
        for (SwerveModuleState state : moduleStates) {
            averageVelocity += Math.abs(state.speedMetersPerSecond);
            averageFalconVelocity += Math.abs(Conversions.MPSToFalcon(
                    state.speedMetersPerSecond,
                    Constants.SwerveConstants.DRIVETRAIN_CONSTANTS.getWheelCircumferenceMeters(),
                    Constants.SwerveConstants.DRIVETRAIN_CONSTANTS.getDriveGearRatio())
            );
        }
        averageVelocity /= moduleStates.length;
        averageFalconVelocity /= moduleStates.length;
        xVelocities.add(averageVelocity);
        xFalconVelocities.add(averageFalconVelocity);

        double tempSum = 0;
        for(SwerveModuleBase mod:drivetrain.getSwerveMods()) {
            tempSum += Math.abs(mod.getTick());
        }
        travelTicks = tempSum / drivetrain.getSwerveMods().length;
    };

    private final Notifier n = new Notifier(r);

    @Override
    public void initialize() {
        prepTimer.reset();
        timer.reset();
        drivetrain.empty();
        System.out.println("--- Linear Characterization of the Drivetrain Starts ---");
        prepTimer.start();
        n.startPeriodic(0.01);
    }

    @Override
    public void end(boolean interrupted) {
        n.stop();
        drivetrain.stopMovement();
        drivetrain.normal();

        System.out.println("--- Linear Characterization of the Drivetrain Ends ---");
        System.out.println("Total Time Taken: " + timer.get());
        prepTimer.reset();
        timer.stop();

        if(xVelocities.size() == 0 || yVoltages.size() == 0 || xFalconVelocities.size() == 0) return;

        PolynomialRegression regression = new PolynomialRegression(
                xVelocities.stream().mapToDouble(Math::abs).toArray(),
                yVoltages.stream().mapToDouble(Math::abs).toArray(), 1);
        System.out.println("Fit R2: " + regression.R2());
        System.out.println("Drivetrain KS: " + regression.beta(0) + " V");
        System.out.println("Drivetrain kV: " + regression.beta(1) + " V / ms^{-1}");
        System.out.println(
                "Converted Module kV: "
                        + regression.beta(1) / Constants.SwerveConstants.DRIVETRAIN_CONSTANTS.getDriveGearRatio()
                        * Constants.SwerveConstants.DRIVETRAIN_CONSTANTS.getWheelCircumferenceMeters()
                        + " V / rps");

        PolynomialRegression regressionFalcon = new PolynomialRegression(
                xFalconVelocities.stream().mapToDouble(Math::abs).toArray(),
                yVoltages.stream().mapToDouble(Math::abs).toArray(), 1);
        System.out.println(
                "Converted Module kV in Falcon Units:"
                        + 1024.0 * regressionFalcon.beta(0) + "Falcon Output Units / Falcon Encoder Units / 100ms");
        System.out.println(
                "Travelled Ticks: " + travelTicks
        );
    }

    @Override
    public boolean isFinished() {
        return (startVoltage + deltaVoltage * timer.get()) >= maxVoltage;
    }
}