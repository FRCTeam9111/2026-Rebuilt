// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

 // private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
    // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
public RobotContainer() { 
    // Configure the button bindings
    configureButtonBindings();

    configureAutoChooser();   // add autonomous options

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureAutoChooser() {
    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Example Auto", Autos.exampleAuto(m_robotDrive, ballSubsystem));
    autoChooser.addOption("Trajectory Example Auto)", Autos.exampleTrajectoryAuto(m_robotDrive, ballSubsystem));
    }

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR2.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    /*Using CommandXboxController, the above code can be rewritten as:
     * 
     * While the right trigger on the operator controller is held, drive in an X
     * driverController.rightTrigger()
        .whileTrue(m_robotDrive.run((() -> m_robotDrive.setX())));

        driverController.start()
        .whileTrue(m_robotDrive.runOnce((() -> m_robotDrive.zeroHeading())));
     */

    // While the left bumper on operator controller is held, intake Fuel
    driverController.leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    driverController.rightBumper()
    .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.launch(), () -> ballSubsystem.stop()));
   /*  .whileTrue(ballSubsystem.spinUpCommand().withTimeout(FuelConstants.Launcher.SPINUP_SECONDS)
            .andThen(ballSubsystem.launchCommand())
            .finallyDo(() -> ballSubsystem.stop()));*/
        
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    driverController.a()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));



// at top: import edu.wpi.first.wpilibj2.command.button.POVButton;

    /*driverController.povUp()
        .onTrue(elevator.setTargetPositionCommand(ElevatorPosition.CORAL_L2));

    driverController.povDown()
        .onTrue(elevator.setTargetPositionCommand(ElevatorPosition.BOTTOM));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();   
  }
}
