package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("myCamera");
    private final PhotonPoseEstimator poseEstimator;

    // This is the variable we will update every 20ms
    private Pose2d latestFieldPose = new Pose2d();
    private boolean hasTarget = false;

    // Add the drivetrain as a variable so the vision system can talk to it
    private final DriveSubsystem drivetrain;

    public VisionSubsystem(DriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        // 1. Load the official WPILib Field Map
        // (WPILib updates this enum every year, e.g., k2024Crescendo, k2025Reefscape, etc.)
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 

        // 2. Define where the camera is on your robot
        // Example: Camera is 10 inches (0.254m) forward, dead center, 20 inches (0.508m) high, tilted back 30 degrees
        Transform3d robotToCam = new Transform3d(
                new Translation3d(0.254, 0.0, 0.508), 
                new Rotation3d(0, Math.toRadians(-30), 0)
        );

        // 3. Create the Pose Estimator
        // MULTI_TAG_PNP_ON_COPROCESSOR is the most accurate strategy. 
        // It uses the Raspberry Pi to calculate the 3D geometry before sending it to Java.
        poseEstimator = new PhotonPoseEstimator(
                fieldLayout, 
                robotToCam
        );
    }

    @Override
    public void periodic() {
        // UPDATED: Grab the latest camera frame first
        var pipelineResult = camera.getLatestResult();

        // Then, feed that frame into the estimator using your preferred strategy.
        // This runs the heavy multi-tag math!
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(pipelineResult);

        if (estimatedPose.isPresent()) {
            hasTarget = true;
            latestFieldPose = estimatedPose.get().estimatedPose.toPose2d();
            
            // Push the data to the drivetrain instantly!
            drivetrain.addVisionMeasurement(
                latestFieldPose, 
                estimatedPose.get().timestampSeconds
            );
            
        } else {
            hasTarget = false;
        }
    }

    // --- GETTERS ---
    
    public boolean hasTarget() {
        return hasTarget;
    }

    /** Returns your exact X/Y coordinates and rotation on the field */
    public Pose2d getLatestFieldPose() {
        return latestFieldPose;
    }

    /** * Replaces your old math! 
     * If you want to know how far you are from exactly X=16.0 meters, Y=5.5 meters...
     */
    public double getDistanceToHub() {
        // Example coordinates of the target on the field map
        Pose2d hubLocation = new Pose2d(16.0, 5.5, new Rotation2d()); 
        
        // WPILib calculates the physical distance between your current pose and the target pose
        return latestFieldPose.getTranslation().getDistance(hubLocation.getTranslation());
    }
}