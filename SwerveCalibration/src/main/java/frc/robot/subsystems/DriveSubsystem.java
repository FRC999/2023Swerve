// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  
  private int currentTestModule = 1;

  private String[] testModuleDescription = { "", "Motors 1-2", "Motors 3-4", "Motors 5-6", "Motors 7-8"};

  public WPI_TalonSRX[] motor = new WPI_TalonSRX[9];


  public DriveSubsystem() {
    for (int i=1;i<9;i++) {
      motor[i] = new WPI_TalonSRX(i);
      motor[i].configFactoryDefault();
      motor[i].setSafetyEnabled(false);
      if (i%2 != 0) {   // Odd numbers are drive motors
        motor[i].configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
      } else {   // even motors are relative
        motor[i].configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
      }
    }

  }

  public void zeroDriveEncoders() {  // zero encoders on master mmotor controllers of the drivetrain
    motor[(currentTestModule-1)*2+1].setSelectedSensorPosition(0);
    motor[(currentTestModule-1)*2+2].setSelectedSensorPosition(0);
  }

  public int getDriveEncoder(int motorNumber) {
     return (int) motor[motorNumber].getSelectedSensorPosition();
  }

  public int getDriveEncoderSpeed(int motorNumber) {
    return (int) motor[motorNumber].getSelectedSensorVelocity();
  }

  public String getCurrentModule() {
    return testModuleDescription[currentTestModule];
  }

  public void nextCurrentModule() {
    if (++currentTestModule > 4) {currentTestModule=1;}
  }

  public void previousCurrentModule() {
    if (--currentTestModule < 1 ) {currentTestModule=4;}
  }

  public void runCurrentMotors() {
    motor[(currentTestModule-1)*2+1].set(TalonSRXControlMode.PercentOutput,RobotContainer.driveStick.getY()*(-1));
    motor[(currentTestModule-1)*2+2].set(TalonSRXControlMode.PercentOutput,RobotContainer.driveStick.getX());
  }

  public void stopCurrentMotors() {
    motor[(currentTestModule-1)*2+1].set(TalonSRXControlMode.PercentOutput,0);
    motor[(currentTestModule-1)*2+2].set(TalonSRXControlMode.PercentOutput,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
