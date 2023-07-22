package frc.robot.states;

import edu.wpi.first.math.interpolation.Interpolatable;
import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class ShootingParameters implements Interpolatable<ShootingParameters> {
    private double velocityRpm;
    private double backboardAngleDegree;

    public ShootingParameters plus(ShootingParameters value) {
        return new ShootingParameters(
                velocityRpm + value.getVelocityRpm(),
                backboardAngleDegree + value.getBackboardAngleDegree()
        );
    }

    public ShootingParameters minus(ShootingParameters value) {
        return new ShootingParameters(
                velocityRpm - value.getVelocityRpm(),
                backboardAngleDegree - value.getBackboardAngleDegree()
        );
    }

    public ShootingParameters times(double multiplier) {
        return new ShootingParameters(
                velocityRpm * multiplier,
                backboardAngleDegree * multiplier
        );
    }

    @Override
    public ShootingParameters interpolate(ShootingParameters endValue, double t) {
        return endValue.minus(this).times(t).plus(this);
    }

    @Override
    public String toString() {
        return String.format("Velocity: %.2f RPM; Hood: %.2f Deg", velocityRpm, backboardAngleDegree);
    }
}
