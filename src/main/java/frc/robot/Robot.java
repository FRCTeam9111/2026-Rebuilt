// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  
  Pose3d robotPose;
  boolean launcherSpinCmd;

  public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);


  // Create the driver's controller for the teleop period. 
  // This is used in the teleopPeriodic method to read driver inputs and control the robo for autoalignment.
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // 1. Create the camera object
    PhotonCamera camera = new PhotonCamera("frontCamera");

    // 3. Constants for "Aiming"
    final double LINEAR_P = 0.1; // Speed to move forward
    final double ANGULAR_P = 0.05; // Speed to turn

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final PIDController aimPID = new PIDController(DriveConstants.VISION_TURN_kP, 0, 0);

  public final DriveSubsystem m_swerveDrive = m_robotContainer.getDriveSubsystem(); // may need to put this into robotInit

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

        Pose3d robotPose = new Pose3d();

      // Calculate drivetrain commands from Joystick values
        double forward = -m_driverController.getLeftY();   // normalized [-1..1]
        double strafe  = -m_driverController.getLeftX();   // normalized [-1..1]
        double rotation = -m_driverController.getRightX(); // normalized [-1..1]

        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;
        
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();

                        targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        0.5, // Measured with a tape measure, or in CAD.
                                        1.435, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));
                        targetVisible = true;
                    }
                }
            }
        }

        // Auto-align when requested
        if (m_driverController.getAButton() && targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.

            // aimPID expects radians (or degrees consistently) — pick one and be consistent
            double targetYawRad = Math.toRadians(targetYaw);
            // If we treat current yaw as 0 (camera-centered), the controller can compute directly:
            double rotOut = aimPID.calculate(0.0, targetYawRad); // output in same units as P (radians * K)
            rotation = MathUtil.clamp(-rotOut, -1.0, 1.0);

            double autoSpeed = (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP;
            forward = (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * Constants.Swerve.kMaxLinearSpeed;
        }

    PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();


    if (kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
      AprilTagFieldLayout aprilTagFieldLayout;
            Transform3d cameraToRobot;
      robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
    }
      
      if(robotPose.X() < 1.5){
      // Near blue alliance wall, start spinning the launcher wheel
      launcherSpinCmd = true;
    } else {
      // Far away, no need to run launcher.
      launcherSpinCmd = false;
    }

      // Use fieldRelative = true if you want joystick inputs field-relative; false for robot-relative.
      boolean fieldRelative = true;
      m_swerveDrive.drive(forward, strafe, rotation, fieldRelative);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
