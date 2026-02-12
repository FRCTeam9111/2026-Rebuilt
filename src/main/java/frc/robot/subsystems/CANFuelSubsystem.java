// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.FuelConstants.Feeder;
import frc.robot.Constants.FuelConstants.Intake;
import frc.robot.Constants.FuelConstants.Launcher;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final SparkMax launcherRoller;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
  // create motors for each of the motors on the launcher mechanism
  intakeLauncherRoller = new SparkMax(Launcher.INTAKE_MOTOR_ID, MotorType.kBrushless);
  launcherRoller = new SparkMax(Launcher.LAUNCHER_ROLLER_MOTOR_ID, MotorType.kBrushless); // this is the orange roller at the top
  feederRoller = new SparkMax(Feeder.MOTOR_ID, MotorType.kBrushless);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
  SmartDashboard.putNumber("Intaking feeder roller value", Feeder.INTAKE_VOLTS);
  SmartDashboard.putNumber("Intaking intake roller value", Intake.ROLLER_VOLTS);
  SmartDashboard.putNumber("Launching feeder roller value", Feeder.LAUNCH_VOLTS);
  SmartDashboard.putNumber("Launching launcher roller value", Launcher.SHOOTER_VOLTS);
  SmartDashboard.putNumber("Spin-up feeder roller value", Feeder.SPINUP_VOLTS);

    

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
  SparkMaxConfig feederConfig = new SparkMaxConfig();
  feederConfig.smartCurrentLimit(Feeder.CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
  launcherConfig.smartCurrentLimit(Launcher.CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig topLauncherConfig = new SparkMaxConfig();
    topLauncherConfig.inverted(true);
  topLauncherConfig.smartCurrentLimit(Launcher.CURRENT_LIMIT);
  topLauncherConfig.follow(Launcher.INTAKE_MOTOR_ID);
    launcherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  

  // A method to set the rollers to values for intaking
  public void intake() {
  feederRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", Feeder.INTAKE_VOLTS));
  intakeLauncherRoller
    .setVoltage(-1 * SmartDashboard.getNumber("Intaking intake roller value", Intake.ROLLER_VOLTS));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
  feederRoller
    .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", Feeder.INTAKE_VOLTS));
  intakeLauncherRoller
    .setVoltage(-1 * SmartDashboard.getNumber("Intaking launcher roller value", Intake.ROLLER_VOLTS));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
  feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", Feeder.LAUNCH_VOLTS));
  intakeLauncherRoller
  .setVoltage(SmartDashboard.getNumber("Intaking launcher roller value", Launcher.SHOOTER_VOLTS));
  launcherRoller.setVoltage(SmartDashboard.getNumber("Launcher roller value", Launcher.SHOOTER_VOLTS));
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
  feederRoller
    .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", Feeder.SPINUP_VOLTS));
  intakeLauncherRoller
    .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", Launcher.SHOOTER_VOLTS));
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
