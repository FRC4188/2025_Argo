package frc.robot.subsystems.tracking;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TrackingIOPhoton implements TrackingIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  public TrackingIOPhoton(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(TrackingIOInputs inputs) {
    inputs.camConnected = camera.isConnected();

    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.hasTarget = true;
      } else {
        inputs.hasTarget = false;
      }

      // getting algae target
      if (result.hasTargets()) {
        var mx = 1.3;
        var bx = 0;
        var my = 1.3;
        var by = 0;
        PhotonTrackedTarget target = result.getBestTarget();
        var xDist = target.getPitch() * mx;
        var yDist = target.getYaw() * my;
        inputs.targetTransform =
            new Transform2d(new Translation2d(xDist, yDist), Rotation2d.fromDegrees(target.getYaw()));
      }
    }
  }
}
