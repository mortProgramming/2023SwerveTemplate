package org.mort11.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private static Limelight limelight;

    private PhotonCamera camera;
    private PhotonPipelineResult cameraResult;

    public Limelight() {
        camera = new PhotonCamera("photoncamera");
        cameraResult = camera.getLatestResult();
    }

    public boolean getHasTargets() {
        return cameraResult.hasTargets();
    }

    public double getTargetPitch() {
        return cameraResult.getBestTarget().getPitch();
    }

    public double getTargetYaw() {
        return cameraResult.getBestTarget().getYaw();
    }

    public double getTargetArea() {
        return cameraResult.getBestTarget().getArea();
    }

    public double getTargetSkew() {
        return cameraResult.getBestTarget().getSkew();
    }

    public double getTargetPose() {
        return cameraResult.getBestTarget().getPoseAmbiguity();
    }

    public int getTargetId() {
        return cameraResult.getBestTarget().getFiducialId();
    }

    public static Limelight getInstance() {
        if (limelight == null) {
            limelight = new Limelight();
        }
        return limelight;
    }
}
