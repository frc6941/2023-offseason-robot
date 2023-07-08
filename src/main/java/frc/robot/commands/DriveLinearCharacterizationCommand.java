package frc.robot.commands;

import java.util.ArrayList;

import com.team254.lib.util.PolynomialRegression;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveLinearCharacterizationCommand extends CommandBase {
    private Swerve mDrivetrain;
    private double startVoltage;
    private double deltaVoltage;
    private double prepTime = 3.0;

    private Timer prepTimer = new Timer();
    private Timer timer = new Timer();

    private ArrayList<Double> yVoltages = new ArrayList<>();
    private ArrayList<Double> xVelocities = new ArrayList<>();

    public DriveLinearCharacterizationCommand(Swerve mDrivetrain, double startVoltage, double deltaVoltage) {
        this.mDrivetrain = mDrivetrain;
        this.startVoltage = startVoltage;
        this.deltaVoltage = deltaVoltage;
    }

    Runnable r = () -> {
        if (prepTimer.get() < prepTime) {
            timer.stop();
            SwerveModuleState individualState = new SwerveModuleState(
                    0.0, new Rotation2d());
            mDrivetrain.setModuleStates(new SwerveModuleState[] {
                    individualState, individualState, individualState, individualState
            }, true, true);
        } else {
            timer.start();
            double targetVoltage = startVoltage + deltaVoltage * timer.get();

            SwerveModuleState individualState = new SwerveModuleState(targetVoltage / 12.0, new Rotation2d());
            mDrivetrain.setModuleStates(new SwerveModuleState[] {
                    individualState, individualState, individualState, individualState
            }, true, true);
            yVoltages.add(targetVoltage);

            SwerveModuleState[] moduleStates = mDrivetrain.getModuleStates();
            double averageVelocity = 0.0;
            for (SwerveModuleState state : moduleStates) {
                averageVelocity += Math.abs(state.speedMetersPerSecond);
            }
            averageVelocity /= moduleStates.length;
            xVelocities.add(averageVelocity);
        }
    };

    Notifier n = new Notifier(r);

    @Override
    public void initialize() {
        prepTimer.reset();
        timer.reset();
        mDrivetrain.empty();
        System.out.println("--- Linear Characterization of the Drivetrain Starts ---");
        prepTimer.start();
        n.startPeriodic(0.01);
    }

    @Override
    public void end(boolean interrupted) {
        n.stop();
        mDrivetrain.normal();

        System.out.println("--- Linear Characterization of the Drivetrain Ends ---");
        System.out.println("Total Time Taken: " + timer.get());
        prepTimer.reset();
        timer.stop();

        PolynomialRegression regression = new PolynomialRegression(
                xVelocities.stream().mapToDouble(d -> Math.abs(d)).toArray(),
                yVoltages.stream().mapToDouble(d -> Math.abs(d)).toArray(), 1);
        System.out.println("Fit R2: " + regression.R2());
        System.out.println("Drivetrain KS: " + regression.beta(0) + " V");
        System.out.println("Drivetrain kV: " + regression.beta(1) + " V / ms^{-1}");
        System.out.println(
            "Converted Module kV: " 
            + regression.beta(1) / Constants.SwerveConstants.DRIVETRAIN_CONSTANTS.getDriveGearRatio()
            * Constants.SwerveConstants.DRIVETRAIN_CONSTANTS.getWheelCircumferenceMeters()
            + " V / rps");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}