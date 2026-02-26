// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.DriveSubsystem; 

public final class Autos {
  
  private final static DriveSubsystem m_robotDrive = new DriveSubsystem();

  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(DriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
    
    /*
     * return new SequentialCommandGroup(
        // Drive backwards for .25 seconds. The driveArcadeAuto command factory
        // creates a command which does not end which allows us to control
        // the timing using the withTimeout decorator
        m_robotDrive.driveForward(m_robotDrive, 0.5, 0, 0, true).withTimeout(.25),
        // Stop driving. This line uses the regular driveArcade command factory so it
        // ends immediately after commanding the motors to stop
        m_robotDrive.driveForward(m_robotDrive, 0.0, 0, 0, true),
        // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
        // total of 10 seconds
        ballSubsystem.spinUpCommand().withTimeout(1),
        ballSubsystem.launchCommand().withTimeout(9),
        // Stop running the launcher
        ballSubsystem.runOnce(() -> ballSubsystem.stop()));
     * 
     */
    
      return new SequentialCommandGroup(
      // Drive forward for 0.25s using the provided driveSubsystem
      driveSubsystem.driveForward(driveSubsystem, 0.5, 0, 0, true).withTimeout(0.25),
      // Stop
      driveSubsystem.driveForward(driveSubsystem, 0.0, 0, 0, true),
      // Spin up + launch
      ballSubsystem.spinUpCommand().withTimeout(1),
      ballSubsystem.launchCommand().withTimeout(9),
      // Stop launcher
      ballSubsystem.runOnce(ballSubsystem::stop)
    );

    /*
     * return Commands.run(
       () -> m_robotDrive.drive(1, 1, 0, true), driveSubsystem).withTimeout(0.25);
     * 
     */
  }  
  public static final Command exampleTrajectoryAuto(DriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
      
}

