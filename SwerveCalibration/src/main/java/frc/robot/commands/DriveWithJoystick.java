// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveWithJoystick extends CommandBase {
  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // The relative encoders should already be set to 0
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set PID for the turn motors and Power for the drive motors
    RobotContainer.driveSubsystem.turnWheelsToJoystick(-RobotContainer.driveStick.getY(), RobotContainer.driveStick.getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
