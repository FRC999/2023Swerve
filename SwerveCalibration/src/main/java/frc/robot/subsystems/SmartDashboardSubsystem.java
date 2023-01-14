// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {}

  public void updateDriveSubsystemTelemetry() {

    for (int i=1;i<9;i++) {
      SmartDashboard.putNumber("Motor " + i + " Encoder Value", RobotContainer.driveSubsystem.getDriveEncoder(i));
      SmartDashboard.putNumber("Motor " + i + " Encoder Speed", RobotContainer.driveSubsystem.getDriveEncoderSpeed(i));

      if (i%2==0){
        SmartDashboard.putNumber("Motor " + i + " Abs Encoder Value", RobotContainer.driveSubsystem.getDriveAbsEncoder(i));
      }
    }

    SmartDashboard.putString("Module:",RobotContainer.driveSubsystem.getCurrentModule());

    //SmartDashboard.putNumber("Left1 Encoder Value", RobotContainer.driveSubsystem.getLeftEncoder(1));
  }

  public void updateAllDisplays() {
    updateDriveSubsystemTelemetry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}
