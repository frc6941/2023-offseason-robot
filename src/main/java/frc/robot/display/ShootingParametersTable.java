package frc.robot.display;

import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.states.ShootingParameters;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;
import lombok.Synchronized;
import org.frcteam6328.utils.TunableNumber;

import java.util.*;
import java.util.Map.Entry;

public class ShootingParametersTable {
    private final NavigableMap<Double, ShootingParameters> interpolatingTable = new TreeMap<>();
    private final TunableNumber fenderShotAngle = new TunableNumber("P Fender BBA", Constants.HoodConstants.HOOD_MINIMUM_ANGLE);
    private final TunableNumber fenderShotVelocity = new TunableNumber("P Fender FWV", 1750);
    private final TunableNumber customShotAngle = new TunableNumber("P Custom BBA", 20.0);
    private final TunableNumber customShotVelocity = new TunableNumber("P Custom FWV", 500.0);

    @Getter @Setter
    private double currentTuningShootingParametersDistance = 2.0;

    private static ShootingParametersTable instance;

    public static ShootingParametersTable getInstance() {
        if (instance == null) {
            instance = new ShootingParametersTable();
        }
        return instance;
    }

    private ShootingParametersTable() {
        loadParameter(2.0, 1850, 10.0);
        loadParameter(2.5, 1950, 15.0);
        loadParameter(3.0, 1950, 21.0);
        loadParameter(3.5, 2075, 22.3);
        loadParameter(4.0, 2125, 23.7);
        loadParameter(4.5, 2200, 28.4);
        loadParameter(5.0, 2325, 28.7);
        loadParameter(5.5, 2450, 29.0);
        loadParameter(6.0, 2550, 29.5);
        loadParameter(6.5, 2700, 29.7);
        loadParameter(7.0, 2825, 30.0);
    }

    private void loadParameter(double distance, double velocityRpm, double backboardAngleDegrees) {
        interpolatingTable.put(distance, new ShootingParameters(velocityRpm, backboardAngleDegrees));
    }

    @Synchronized
    public void update() {
        if(!Constants.TUNING) return;

        SmartDashboard.putNumber("Current Tuning Distance", currentTuningShootingParametersDistance);
        ShootingParameters currentParameters = getParameters(currentTuningShootingParametersDistance);
        SmartDashboard.putNumber("Current Tuning RPM", currentParameters.getVelocityRpm());
        SmartDashboard.putNumber("Current Tuning Hood", currentParameters.getBackboardAngleDegree());

        StringBuilder result = new StringBuilder();
        for(Double distance: interpolatingTable.navigableKeySet()) {
            ShootingParameters parameters = interpolatingTable.get(distance);
            result.append(String.format("Distance: %s, Rpm: %s, Angle: %s ;;;", distance, parameters.getVelocityRpm(), parameters.getBackboardAngleDegree()));
        }

        SmartDashboard.putString("Current Tuning Result (For Copy)", result.toString());
    }


    public void updateSpecific(double distance, double newFlywheelRpm, double newBackboardAngle) {
        for (Double d : interpolatingTable.navigableKeySet()) {
            if(d == distance) {
                interpolatingTable.put(distance, new ShootingParameters(newFlywheelRpm, newBackboardAngle));
            }
        }
    }

    public ShootingParameters getParameters(double distance) {
        if (distance <= interpolatingTable.firstKey()) {
            return interpolatingTable.firstEntry().getValue();
        }

        if (distance >= interpolatingTable.lastKey()) {
            return interpolatingTable.lastEntry().getValue();
        }

        Entry<Double, ShootingParameters> floor = interpolatingTable.floorEntry(distance);
        Entry<Double, ShootingParameters> ceiling = interpolatingTable.ceilingEntry(distance);

        if(Objects.equals(ceiling.getKey(), floor.getKey())) return floor.getValue();

        return floor.getValue().interpolate(
                ceiling.getValue(),
                (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey())
        );
    }

    public ShootingParameters getFenderShotParameters() {
        return new ShootingParameters(fenderShotVelocity.get(), fenderShotAngle.get());
    }

    public ShootingParameters getCustomShotParameters() {
        return new ShootingParameters(customShotVelocity.get(), customShotAngle.get());
    }
}
