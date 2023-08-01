package frc.robot.commands;

import com.team254.lib.util.PolynomialRegression;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import org.frcteam1678.lib.math.Conversions;

import java.util.ArrayList;

public class CharacterizeMotorCommand extends CommandBase {
    private double startVoltage;
    private double deltaVoltage;
    private final double maxVoltage;
    private final double prepTime = 3.0;

    private final Timer prepTimer = new Timer();
    private final Timer timer = new Timer();

    private final ArrayList<Double> yVoltages = new ArrayList<>();
    private final ArrayList<Double> xFalconVelocities = new ArrayList<>();
    private Shooter shooter;

    public CharacterizeMotorCommand(Shooter shooter, double startVoltage, double deltaVoltage, double maxVoltage) {
        this.shooter = shooter;
        this.startVoltage = startVoltage;
        this.deltaVoltage = deltaVoltage;
        this.maxVoltage = maxVoltage;
    }

    private final Runnable r = () -> {
        if (prepTimer.get() < prepTime) {
            timer.stop();
        } else {
            timer.start();
            double targetVoltage = startVoltage + deltaVoltage * timer.get();

            shooter.setShooterPercentage(targetVoltage / 12.0);

            yVoltages.add(targetVoltage);
            xFalconVelocities.add(
                    Conversions.RPMToFalcon(shooter.getShooterRPM(), Constants.ShooterConstants.SHOOTER_GEAR_RATIO)
            );
        }
    };

    private final Notifier n = new Notifier(r);

    @Override
    public void initialize() {
        prepTimer.reset();
        timer.reset();
        System.out.println("--- Linear Characterization of Motor Starts ---");
        prepTimer.start();
        n.startPeriodic(0.01);
    }

    @Override
    public void end(boolean interrupted) {
        n.stop();

        System.out.println("--- Linear Characterization of Motor Ends ---");
        System.out.println("Total Time Taken: " + timer.get());
        prepTimer.reset();
        timer.stop();

        if(yVoltages.size() == 0 || xFalconVelocities.size() == 0) return;

        PolynomialRegression regressionFalcon = new PolynomialRegression(
                xFalconVelocities.stream().mapToDouble(Math::abs).toArray(),
                yVoltages.stream().mapToDouble(Math::abs).toArray(), 1);
        System.out.println(
                "Converted Module kV in Falcon Units:"
                        + 1024.0 * regressionFalcon.beta(0) + "Falcon Output Units / Falcon Encoder Units / 100ms");
    }

    @Override
    public boolean isFinished() {
        return (startVoltage + deltaVoltage * timer.get()) >= maxVoltage;
    }
}