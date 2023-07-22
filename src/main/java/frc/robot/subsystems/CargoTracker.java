package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import lombok.Getter;
import org.frcteam6941.looper.Updatable;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class CargoTracker implements Updatable, Subsystem {
    private final PhotonCamera cargoTracker;

    private static CargoTracker instance;
    private PhotonPipelineResult result;
    @Getter
    private Optional<Transform2d> cameraToTarget = Optional.empty();

    public static CargoTracker getInstance() {
        if (instance == null) {
            instance = new CargoTracker();
        }
        return instance;
    }

    private CargoTracker() {
        cargoTracker = new PhotonCamera(Constants.CargoTrackerConstants.PIPELINE_NAME);
    }

    @Override
    public void read(double time, double dt) {
        result = cargoTracker.getLatestResult();
        if(result.hasTargets()) {
            cameraToTarget = Optional.of(result.getBestTarget().getCameraToTarget());
        } else {
            cameraToTarget = Optional.empty();
        }
    }

    @Override
    public void telemetry() {
        cameraToTarget.ifPresent(transform2d -> SmartDashboard.putNumberArray("Cargo Tracer", new double[] {transform2d.getX(), transform2d.getY()}));
    }
}
