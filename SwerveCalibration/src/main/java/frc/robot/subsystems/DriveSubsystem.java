// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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

  // Absolute encoder setup
  final boolean kDiscontinuityPresent = true;
	final int kBookEnd_0 = 910;		/* 80 deg */
	final int kBookEnd_1 = 1137;	/* 100 deg */
  /* Nonzero to block the config until success, zero to skip checking */
  final int kTimeoutMs = 30;
  final int[] wheelZeroAngleValues = {0, 0, 3489, 0, 3307, 0, 433, 0, 3268};
  public final int turnPIDTolerance = 3; 

  final int clicksSRXPerFullRotation = 4096;

  public boolean wheelsCalibrated = false;
  public final double drivePowerThreshold = 0.05; // Joystick deadband
  public double currentPIDAngle = 0;
  public double powerReducer = 0.3;

  public DriveSubsystem() {

    for (int i=1;i<9;i++) {
      motor[i] = new WPI_TalonSRX(i);
      motor[i].configFactoryDefault();
      motor[i].setSafetyEnabled(false);

      if (i%2 != 0) {   // Odd numbers are drive motors
        motor[i].configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
      } 
      
      else {   // even motors are relative
        motor[i].configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);

        // Absolute encoder configuration
        initQuadrature(i);

        double selSenPos = motor[i].getSelectedSensorPosition(0);
		    int pulseWidthWithoutOverflows = 
				    motor[i].getSensorCollection().getPulseWidthPosition() & 0xFFF;

        /**
         * Display how we've adjusted PWM to produce a QUAD signal that is
         * absolute and continuous. Show in sensor units and in rotation
         * degrees.
         */
        System.out.print("pulseWidPos:" + pulseWidthWithoutOverflows +
                "   =>    " + "selSenPos:" + selSenPos);
        System.out.print("      ");
        System.out.print("pulseWidDeg:" + ToDeg(pulseWidthWithoutOverflows) +
                "   =>    " + "selSenDeg:" + ToDeg(selSenPos));
        System.out.println();

        this.configureSimpleMagic(i);

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

  public int getDriveAbsEncoder(int motorNumber) {
    return (int) motor[motorNumber].getSensorCollection().getPulseWidthPosition() & 0xFFF;
 }

 public void setEncoderforWheelCalibration(int motorNumber){
  int difference = getDriveAbsEncoder(motorNumber) -  wheelZeroAngleValues[motorNumber]; 
  int encoderSetting = 0; 

  //System.out.println("I0 d " + difference);

  if(difference < 0){
    difference += clicksSRXPerFullRotation; 
  }

  //System.out.println("I1 d " + difference);

  if(difference <= clicksSRXPerFullRotation/2){
    encoderSetting = difference; 

  }
  else{
    encoderSetting = difference - clicksSRXPerFullRotation; 
  }

  motor[motorNumber].setSelectedSensorPosition(encoderSetting);

  System.out.println("Set encoder for motor " + motorNumber + " to " + encoderSetting);

 }

 public void turnWheelsToZero(int i) {
   motor[i].set(TalonSRXControlMode.MotionMagic, 0);
   System.out.println("going to 0");
 }

 public void initQuadrature(int motorNumber) { // Set absolute encoders
   int pulseWidth = motor[motorNumber].getSensorCollection().getPulseWidthPosition();

   if (kDiscontinuityPresent) {

     /* Calculate the center */
     int newCenter;
     newCenter = (kBookEnd_0 + kBookEnd_1) / 2;
     newCenter &= 0xFFF;

     /**
      * Apply the offset so the discontinuity is in the unused portion of
      * the sensor
      */
     pulseWidth -= newCenter;
   }

   /**
    * Mask out the bottom 12 bits to normalize to [0,4095],
    * or in other words, to stay within [0,360) degrees
    */
   pulseWidth = pulseWidth & 0xFFF;

   /* Update Quadrature position */
   motor[motorNumber].getSensorCollection().setQuadraturePosition(pulseWidth, kTimeoutMs);

 }

  /**
	 * @param units CTRE mag encoder sensor units 
	 * @return degrees rounded to tenths.
	 */
	String ToDeg(double units) {
		double deg = units * 360.0 / 4096.0;

		/* truncate to 0.1 res */
		deg *= 10;
		deg = (int) deg;
		deg /= 10;

		return "" + deg;
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

  public void stopMotor(int motorNumber) {
    motor[motorNumber].set(TalonSRXControlMode.PercentOutput,0);
  }

  public void configureSimpleMagic(int i) {
    motor[i].setSafetyEnabled(false);

    /* Configure motor neutral deadband */
    motor[i].configNeutralDeadband(0.001, 30);

    motor[i].setSensorPhase(false);

    motor[i].setInverted(false);

    /* Set status frame periods to ensure we don't have stale data */
    
    motor[i].setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10,
        30);

    motor[i].setStatusFramePeriod(StatusFrame.Status_10_Targets, 10,
        30);
    //rightmotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20,
    //    30);
    //leftmotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20,
    //    30);

    /**
    * Max out the peak output (for all modes). However you can limit the output of
    * a given PID object with configClosedLoopPeakOutput().
    */
    motor[i].configPeakOutputForward(+1.0, 30);
    motor[i].configPeakOutputReverse(-1.0, 30);
    motor[i].configNominalOutputForward(0, 30);
    motor[i].configNominalOutputReverse(0, 30);    
    
    /* FPID Gains for each side of drivetrain */
    motor[i].selectProfileSlot(0, 0);
    motor[i].config_kP(0, 0.75, 30);
    motor[i].config_kI(0, 0.005, 30);
    motor[i].config_kD(0, 0.01,  30);
    motor[i].config_kF(0, 0, 30);

    motor[i].config_IntegralZone(0, 500,  30);
    motor[i].configClosedLoopPeakOutput(0, 0.5, 30);
    motor[i].configAllowableClosedloopError(0, 5, 30);
  
    motor[i].configClosedLoopPeriod(0, 1,
      30);

  /* Motion Magic Configurations */

  /**
   * Need to replace numbers with real measured values for acceleration and cruise
   * vel.
   */
  motor[i].configMotionAcceleration(6750,
      30);
  motor[i].configMotionCruiseVelocity(6750,
      30);
  motor[i].configMotionSCurveStrength(3);

  }

  public boolean areWheelsCalibrated() {
    return wheelsCalibrated;
  }

  public void turnWheelsToJoystick(double y, double x) { // Set the motors
    double power = Math.sqrt(x*x + y*y);
    for (int i=1;i<9;i+=2) { // Set driving power for the Odd numbered motors
      motor[i].set(TalonSRXControlMode.PercentOutput, (-1)*power*powerReducer);
    }

    if (power > drivePowerThreshold) { // Do not change direction of the robot on the power that is too low

      double driveAngle = Math.toDegrees(Math.atan2(y,x))-90.0; // Joystick direction
      if (driveAngle<0) { driveAngle += 360;} // Now the driveAngle is a clockwise angle from the forward direction being a 0
      
      double currentAngle = currentPIDAngle % 360.0; // Current direction wheels are facing
      if (currentAngle<0) { currentAngle+=360;}

      double diffAngle = currentAngle - driveAngle; 
      double angleIncrement;

      if((Math.abs(diffAngle) >= 180 && currentAngle>driveAngle) || (Math.abs(diffAngle) < 180 && currentAngle < driveAngle)){
        // CounterClockwise, positive increment
        if (driveAngle>=currentAngle) { angleIncrement = driveAngle - currentAngle; }
        else { angleIncrement = driveAngle+360.0 - currentAngle; }
        }
      else{
         // Clockwise, negative increment
         System.out.println("clockwise");
         if (currentAngle >=driveAngle) { angleIncrement = driveAngle - currentAngle; }
         else { angleIncrement = -(currentAngle+360.0 - driveAngle); }
      }

      if (angleIncrement == 360.0 || angleIncrement == -360.0) {angleIncrement=0;}

      System.out.println("C: " + currentPIDAngle + "CA: " + currentAngle + " J: " + driveAngle + " I: " + angleIncrement);

    
      //double difference = driveAngle - currentPIDAngle; // Difference between my previous angle and current one
      //double encoderSettingAngle = 0; 
      //if(difference < 0){difference += 360.0;}
      //if(difference <= 180){ encoderSettingAngle = difference; }
      //else{ encoderSettingAngle = difference - 360;}
    
      for (int i=2;i<9;i+=2) { // Set turning PID for the Even numbered motors
        motor[i].set(TalonSRXControlMode.MotionMagic, (currentPIDAngle+angleIncrement)*clicksSRXPerFullRotation/360.0);
      }
      System.out.println("CPAB: "+ currentPIDAngle);
      currentPIDAngle += angleIncrement;
      System.out.println("CPA: "+ currentPIDAngle);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
