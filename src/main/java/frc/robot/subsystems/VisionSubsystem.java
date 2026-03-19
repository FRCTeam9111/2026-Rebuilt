package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("myCamera");
    
    // Variables that update in your periodic()
    private double targetYaw = 0;
    private double targetRange = 0;
    private boolean hasTarget = false;

   @Override
    public void periodic() {
        // We use getLatestResult() instead of the unread list, it's cleaner!
        var result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            boolean foundTargetTag = false;

            // Loop through all tags the camera can currently see
            for (var target : result.getTargets()) {
                // Check if it's the specific tag we want (ID 7)
                if (target.getFiducialId() == 7) {
                    foundTargetTag = true;
                    targetYaw = target.getYaw();
                    
                    // Do the math to find distance to Tag 7
                    targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                            0.5, // Camera height (Measured in CAD)
                            1.435, // Tag 7 height (From game manual)
                            Units.degreesToRadians(-30.0), // Camera mount angle
                            Units.degreesToRadians(target.getPitch()) // Target pitch
                    );
                    
                    // Break out of the loop since we found what we need
                    break; 
                }
            }
            
            // Update our global boolean
            hasTarget = foundTargetTag;

        } else {
            hasTarget = false;
        }
    }
    
    // Create a "getter" so other parts of your code can ask for the distance
    public double getTargetRange() {
        return targetRange;
    }

    /**
     * COMMAND FACTORY: This creates your auto-align command inline.
     * We pass in the Drivetrain so this subsystem knows what to drive.
     */
    public Command autoAlignCommand(DriveSubsystem drivetrain) {
        return Commands.run(
            () -> {
                // 1. Check if we can see the tag
                if (this.hasTarget) {
                    
                    // 2. Math for driving forward/backward (Clamped!)
                    double rangeError = VisionConstants.VISION_DES_RANGE_m - this.targetRange;
                    double forwardSpeed = MathUtil.clamp(rangeError * VisionConstants.VISION_STRAFE_kP, -1.0, 1.0);
                    
                    // 3. Math for turning (Clamped!)
                    // We want yaw to be 0 (centered).
                    double rotError = 0 - this.targetYaw; 
                    double turnSpeed = MathUtil.clamp(rotError * VisionConstants.VISION_TURN_kP, -1.0, 1.0);

                    // 4. Send it to the drivetrain
                    drivetrain.drive(forwardSpeed, 0, turnSpeed, false);
                } else {
                    // Stop if we lose the target
                    drivetrain.drive(0, 0, 0, false);
                }
            }, 
            drivetrain, this // <--- CRITICAL: Tells WPILib this command requires both subsystems
            
        ).finallyDo(() -> {
            // SAFETY: When the command ends (driver lets go of the button), stop the robot.
            drivetrain.drive(0, 0, 0, false);
        });
    }
}