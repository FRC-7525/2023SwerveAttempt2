package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Camera {
    private PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
    Double[] lastPosArray = new Double[1];

    public Camera(String cameraName) {
        camera = new PhotonCamera(cameraName);

        // Field positions of AprilTags
        Pose3d pose1 = new Pose3d(0,Units.inchesToMeters(-20.9),0,new Rotation3d(0,0, 0));
        Pose3d pose2 = new Pose3d(0,Units.inchesToMeters(-50.25),0,new Rotation3d(0,0, 0));
        Pose3d pose3 = new Pose3d(0,Units.inchesToMeters(-84.7),0,new Rotation3d(0,0, 0));
        
        AprilTag tag1 = new AprilTag(1, pose1);
        AprilTag tag2 = new AprilTag(2, pose2);
        AprilTag tag3 = new AprilTag(3, pose3);

        double fieldLength = 2.8194;
        double fieldWidth = 2.8194;

        List<AprilTag> tags = new ArrayList<>();

        tags.add(tag1);
        tags.add(tag2);
        tags.add(tag3);

        AprilTagFieldLayout layout = new AprilTagFieldLayout(tags, fieldLength, fieldWidth);
        Transform3d cameraToBot = new Transform3d(new Translation3d(0.0, 0.0, 0.5), new Rotation3d(0,0,0)); /* Defines where the camera is relative to the center of the Robot */
        photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, cameraToBot);
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public void periodic() {    
        Optional<EstimatedRobotPose> optionalPoseEstimate = photonPoseEstimator.update();

        Double[] posArray = new Double[0];
        Double[] rotArray = new Double[0];

        if (optionalPoseEstimate.isPresent()) {
            EstimatedRobotPose poseEstimate = optionalPoseEstimate.get();
            posArray = decomposition.DecomposePose3d(poseEstimate.estimatedPose);    
            rotArray = decomposition.Decompose_Rotation_3d(poseEstimate.estimatedPose.getRotation());
        }

        SmartDashboard.putBoolean("AprilTags in View", optionalPoseEstimate.isPresent());
        SmartDashboard.putNumberArray("Pose", posArray);
        SmartDashboard.putNumberArray("Rotations", rotArray);
    }
}
