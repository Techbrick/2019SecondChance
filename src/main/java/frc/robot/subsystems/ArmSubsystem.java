/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Helpers;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualArm;


/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Robot _robot;
  public RobotMap robotMap;
  private TalonSRX mc_arm;
  private VictorSPX mc_armFollower;
  private TalonSRX mc_intake;
  private TalonSRX mc_wrist;
  private Solenoid ejectorSolenoidIn;
  private Solenoid ejectorSolenoidOut;

  // Constants
  private static final int kSlotIdx = 0;
  private static final int kPIDLoopIdx = 0;
  private static final Gains kGains = new Gains((.5*1023)/(4096.0/12), 0.0, 0.0, 0.2, 0, 1.0);
  private static final int length = 5;
  // private static final int wristUpperLimit;
  // private static final int wristLowerLimit;
  // private static final int armUpperLimit;
  // private static final int armLowerLimit;
  
  public ArmSubsystem(Robot r) {  // Initialize the motion magic constants
    _robot = r;
    robotMap = new RobotMap();
    mc_arm = new TalonSRX(RobotMap.armMasterLeft1);
    mc_armFollower = new VictorSPX(RobotMap.armFollowerRight1);
    mc_intake = new TalonSRX(RobotMap.intakeMotor1);
    mc_wrist = new TalonSRX(RobotMap.wristMotor1);

    mc_wrist.setSelectedSensorPosition(0, 0, 10);
    mc_wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0, 10);
    mc_wrist.setSensorPhase(true);
    mc_wrist.configContinuousCurrentLimit(20, 10);
    mc_wrist.enableCurrentLimit(true);

    mc_arm.setSelectedSensorPosition(0, 0, 10);
    mc_arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
    mc_arm.setSensorPhase(false);
    mc_arm.setInverted(false);
    mc_armFollower.setInverted(true);
    mc_armFollower.follow(mc_arm);


    ejectorSolenoidIn = new Solenoid(4);
    ejectorSolenoidOut = new Solenoid(5);
    setHatchEjector(true);

		// /* Set relevant frame periods to be at least as fast as periodic rate */
		// mc_arm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
		// mc_arm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);

		/* Set the peak and nominal outputs */
		mc_arm.configNominalOutputForward(0, 20);
		mc_arm.configNominalOutputReverse(0, 20);
		mc_arm.configPeakOutputForward(1, 20);
		mc_arm.configPeakOutputReverse(-1, 20);

		/* Set Motion Magic gains in slot0 - see documentation */
		mc_arm.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		mc_arm.config_kF(kSlotIdx, kGains.kF, 20);
		mc_arm.config_kP(kSlotIdx, kGains.kP, 20);
		mc_arm.config_kI(kSlotIdx, kGains.kI, 20);
		mc_arm.config_kD(kSlotIdx, kGains.kD, 20);

		/* Set acceleration and vcruise velocity - see documentation */
		mc_arm.configMotionCruiseVelocity(/*15000*/ 350, 20);
		mc_arm.configMotionAcceleration(/*6000*/ 400, 20);
    mc_arm.configAllowableClosedloopError(0, 50, 20);
		/* Zero the sensor */
    mc_arm.setSelectedSensorPosition(0, kPIDLoopIdx, 20);
    resetZero();
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualArm(_robot));
  }

  public void resetZero() { // Resets the encoder
    mc_arm.setSelectedSensorPosition(0, 0, 20);
    mc_wrist.setSelectedSensorPosition(0, 0, 20);
  }

  public int getArmEncoderTicks() {  // Returns the ticks on the encoder
    return mc_arm.getSensorCollection().getPulseWidthPosition();
  }

  public int getWristEncoderTicks()
  {
    return mc_wrist.getSensorCollection().getPulseWidthPosition();
  }

  // public void extensionLimit()
  // {
  //   if((getWristEncoderTicks() >= wristUpperLimit) && (getWristEncoderTicks() <= wristLowerLimit))
  //   {
      
  //   }
  // }

  public void move(int currAngle, int dPos) { // Changes height of arm based on current angle and desired change
    mc_arm.set(ControlMode.MotionMagic, 4096 * 25 * (-currAngle+Math.acos(dPos / -length - Math.cos(currAngle))) / 360);
  }

  public void rotate(int dir) {
    if(dir < 0) {
      mc_arm.set(ControlMode.PercentOutput, 0.30 * dir); // WAS .15
    }
    else {  
      mc_arm.set(ControlMode.PercentOutput, 0.15 * dir); // WAS .15
    }
    
    SmartDashboard.putNumber("Arm Enc Pos", mc_arm.getSelectedSensorPosition(0));
  }

  public void turns(double degrees) { // Turns a certain number of degrees
    mc_arm.set(ControlMode.Position, degrees / RobotMap.ArmTicksToDeg);
    mc_wrist.set(ControlMode.Position, -degrees / RobotMap.ArmTicksToDeg);    // TODO: Get the right coefficient
    SmartDashboard.putNumber("target arm enc", degrees/RobotMap.ArmTicksToDeg);
  }

  public boolean isTurnComplete(double degrees) { // Determines if degrees of current and target match
    return (getArmEncoderTicks() == (degrees / RobotMap.ArmTicksToDeg));
  }
  public void moveToHeight(double height) {
    turns(Math.asin(height / RobotMap.armLength));
    
  }
  public void moveToHeightPreset(int pos) {
    // if(pos < RobotMap.heights.length && pos > 0)  
    //   moveToHeight(RobotMap.heights[pos]);

    mc_arm.set(ControlMode.Position, RobotMap.heights[0][pos]);
    mc_wrist.set(ControlMode.Position, RobotMap.heights[1][pos]);
  }
  public void setIntakeSpeed(double percentSpeed)
  {
      mc_intake.set(ControlMode.PercentOutput, percentSpeed);
  }

  public double getIntakeSpeed()
  {
      return mc_intake.getMotorOutputPercent();
  }

  public void setArmSpeed(double percentSpeed)
  {
    mc_arm.set(ControlMode.PercentOutput, Helpers.DeadbandJoystick(percentSpeed, robotMap));
  }

  public void setWristSpeed(double percentSpeed)
  {
    mc_wrist.set(ControlMode.PercentOutput, Helpers.DeadbandJoystick(percentSpeed, robotMap));
  }

  public void setHatchEjector(boolean isOpen)
  {
      ejectorSolenoidIn.set(isOpen);
      ejectorSolenoidOut.set(!isOpen);
  }

  public boolean getHatchEjectorValue()
  {
     return ejectorSolenoidIn.get();
  }

  public void rotateWrist(int dir) {
    mc_wrist.set(ControlMode.PercentOutput, 0.15 * dir);
    SmartDashboard.putNumber("Wrist Enc Pos", mc_wrist.getSelectedSensorPosition(0));
  }

}
