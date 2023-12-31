package frc.robot.display;

import frc.robot.Constants;
import frc.robot.states.ShootingParameters;
import lombok.AllArgsConstructor;
import lombok.Synchronized;
import org.frcteam6328.utils.TunableNumber;

import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.TreeMap;

public class ShootingParametersTable {
    private final List<ParametersBinding> parameters = new ArrayList<>();
    private final NavigableMap<Double, ShootingParameters> interpolatingTable = new TreeMap<>();
    private final TunableNumber fenderShotAngle = new TunableNumber("P Fender BBA", Constants.HoodConstants.HOOD_MINIMUM_ANGLE);
    private final TunableNumber fenderShotVelocity = new TunableNumber("P Fender FWV", 1750);
    private final TunableNumber customShotAngle = new TunableNumber("P Custom BBA", 20.0);
    private final TunableNumber customShotVelocity = new TunableNumber("P Custom FWV", 500.0);

    private static ShootingParametersTable instance;

    public static ShootingParametersTable getInstance() {
        if (instance == null) {
            instance = new ShootingParametersTable();
        }
        return instance;
    }

    private ShootingParametersTable() {
        loadParameter(2.0, 1850, 10.0);
        loadParameter(2.5, 1950.0, 15.0);
        loadParameter(3.0, 1950, 21.0);
        loadParameter(4.0, 2200, 23.7);
        loadParameter(4.5, 2300, 28.5);
        loadParameter(5.0, 2400, 28.5);
        loadParameter(5.5, 2525, 28.5);
        loadParameter(6.0, 2650, 29.5);
        loadParameter(6.5, 2800, 29.7);
        loadParameter(7.0, 2900, 30);

        readyTuning();
    }

    private void loadParameter(double distance, double velocityRpm, double backboardAngleDegrees) {
        interpolatingTable.put(distance, new ShootingParameters(velocityRpm, backboardAngleDegrees));
    }

    private void readyTuning() {
        int counter = 1;
        for (Double key : interpolatingTable.keySet()) {
            parameters.add(new ParametersBinding(
                    new TunableNumber("P" + counter + " DIS", key),
                    new TunableNumber("P" + counter + " BBA", interpolatingTable.get(key).getBackboardAngleDegree()),
                    new TunableNumber("P" + counter + " FWV", interpolatingTable.get(key).getVelocityRpm())
            ));
            counter++;
        }
    }

    @Synchronized
    public void update() {
        interpolatingTable.clear();
        for (ParametersBinding bind : parameters) {
            interpolatingTable.put(bind.distance.get(), new ShootingParameters(bind.flywheelRpm.get(), bind.backboardAngle.get()));
        }
    }

    @Synchronized
    public ShootingParameters getParameters(double distance) {
        if (distance <= interpolatingTable.firstKey()) {
            return interpolatingTable.firstEntry().getValue();
        }

        if (distance >= interpolatingTable.lastKey()) {
            return interpolatingTable.lastEntry().getValue();
        }

        Entry<Double, ShootingParameters> floor = interpolatingTable.floorEntry(distance);
        Entry<Double, ShootingParameters> ceiling = interpolatingTable.ceilingEntry(distance);

        return floor.getValue().interpolate(
                ceiling.getValue(),
                (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey())
        );
    }

    @Synchronized
    public ShootingParameters getFenderShotParameters() {
        return new ShootingParameters(fenderShotVelocity.get(), fenderShotAngle.get());
    }

    @Synchronized
    public ShootingParameters getCustomShotParameters() {
        return new ShootingParameters(customShotVelocity.get(), customShotAngle.get());
    }

    @AllArgsConstructor
    private class ParametersBinding implements Comparable<ParametersBinding> {
        public TunableNumber distance;
        public TunableNumber backboardAngle;
        public TunableNumber flywheelRpm;

        @Override
        public int compareTo(ParametersBinding bind) {
            if (distance.get() - bind.distance.get() > 0) {
                return 1;
            } else if (distance.get() - bind.distance.get() < 0) {
                return -1;
            } else {
                return 0;
            }
        }
    }
}
