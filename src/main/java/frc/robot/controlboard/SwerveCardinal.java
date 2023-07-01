package frc.robot.controlboard;

public class SwerveCardinal {
    public enum SWERVE_CARDINAL {
        NONE(null),

        FORWARDS(0.0),
        LEFT(270.0),
        RIGHT(90.0),
        BACKWARDS(180.0);

        public final Double degrees;

        SWERVE_CARDINAL(Double degrees) {
            this.degrees = degrees;
        }
    }
}
