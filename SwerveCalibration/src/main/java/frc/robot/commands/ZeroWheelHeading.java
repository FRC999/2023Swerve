// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ZeroWheelHeading extends CommandBase {

  //int motorNumber;
  /** Creates a new ZeroWheelHeading. */
  public ZeroWheelHeading() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int motorNumber = 2; motorNumber<9; motorNumber = motorNumber+2){
      RobotContainer.driveSubsystem.setEncoderforWheelCalibration(motorNumber);
      RobotContainer.driveSubsystem.turnWheelsToZero(motorNumber);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (int motorNumber = 2; motorNumber<9; motorNumber = motorNumber+2){
      RobotContainer.driveSubsystem.stopMotor(motorNumber);
      System.out.println("Motor "+ motorNumber + "calibrated i:" + interrupted);
    }
    RobotContainer.driveSubsystem.wheelsCalibrated = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return 
       Math.abs(RobotContainer.driveSubsystem.getDriveEncoder(2)) <= RobotContainer.driveSubsystem.turnPIDTolerance &&
       Math.abs(RobotContainer.driveSubsystem.getDriveEncoder(4)) <= RobotContainer.driveSubsystem.turnPIDTolerance &&
       Math.abs(RobotContainer.driveSubsystem.getDriveEncoder(6)) <= RobotContainer.driveSubsystem.turnPIDTolerance &&
       Math.abs(RobotContainer.driveSubsystem.getDriveEncoder(8)) <= RobotContainer.driveSubsystem.turnPIDTolerance;
       
  }
}
