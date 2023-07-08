package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.util.PolynomialRegression;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CharacterizeMotorCommand extends CommandBase {
    private double startVoltage;
    private double deltaVoltage;
    private double maxVoltage;
    private double prepTime = 3.0;

    private Timer prepTimer = new Timer();
    private Timer timer = new Timer();

    private ArrayList<Double> yVoltages = new ArrayList<>();
    private ArrayList<Double> xFalconVelocities = new ArrayList<>();
    private TalonFX motor;

    public CharacterizeMotorCommand(TalonFX motor, double startVoltage, double deltaVoltage, double maxVoltage) {
        this.motor = motor;
        this.startVoltage = startVoltage;
        this.deltaVoltage = deltaVoltage;
        this.maxVoltage = maxVoltage;
    }

    Runnable r = () -> {
        if (prepTimer.get() < prepTime) {
            timer.stop();
        } else {
            timer.start();
            double targetVoltage = startVoltage + deltaVoltage * timer.get();

            motor.set(ControlMode.PercentOutput, targetVoltage);
            
            yVoltages.add(targetVoltage);
            xFalconVelocities.add(motor.getSelectedSensorVelocity());
        }
    };

    Notifier n = new Notifier(r);

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

        PolynomialRegression regressionFalcon = new PolynomialRegression(
                xFalconVelocities.stream().mapToDouble(d -> Math.abs(d)).toArray(),
                yVoltages.stream().mapToDouble(d -> Math.abs(d)).toArray(), 1);
        System.out.println(
            "Converted Module kV in Falcon Units:" 
            + 1024.0 * regressionFalcon.beta(0) + "Falcon Output Units / Falcon Encoder Units / 100ms");
    }

    @Override
    public boolean isFinished() {
        return (startVoltage + deltaVoltage * timer.get()) >= maxVoltage;
    }
}