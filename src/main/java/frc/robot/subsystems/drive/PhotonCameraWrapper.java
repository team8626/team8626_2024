package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Quaternion;
public class PhotonCameraWrapper {
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;
    private AprilTagFieldLayout fieldLayout;

    public PhotonCameraWrapper() {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        photonCamera = new PhotonCamera("FrontCamera");

        // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
        fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // Create pose estimator
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, DriveConstants.PhotonVisionConstants.frontCameraRobotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Double[] getBestTarget(){
        
        PhotonPipelineResult result = photonCamera.getLatestResult();
        
        if(result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Pose3d bestTag = (fieldLayout.getTagPose(target.getFiducialId())).get();
            Quaternion tagRotation = bestTag.getRotation().getQuaternion();
            return new Double[] { bestTag.getX(), 
                bestTag.getY(), 
                bestTag.getZ(), 
                tagRotation.getW(), 
                tagRotation.getX(), 
                tagRotation.getY(), 
                tagRotation.getZ() };
        }
        return new Double[] {};
    }
    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     *     the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            System.out.println("didnt work");
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        System.out.println("worked");
        return photonPoseEstimator.update();
    }
}