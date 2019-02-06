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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;


/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Robot _robot;
  private TalonSRX mc_arm;
  private VictorSPX mc_armFollower;
  private TalonSRX mc_intake;
  private TalonSRX mc_wrist;

  // Constants
  private static final int kSlotIdx = 0;
  private static final int kPIDLoopIdx = 0;
  private static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);
  private static final int length = 5;
  
  public ArmSubsystem(Robot r) {  // Initialize the motion magic constants
    _robot = r;
    mc_arm = new TalonSRX(RobotMap.armMasterLeft1);
    mc_armFollower = new VictorSPX(RobotMap.armFollowerRight1);
    mc_intake = new TalonSRX(RobotMap.intakeMotor1);
    mc_wrist = new TalonSRX(RobotMap.wristMotor1);
    mc_arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);

    mc_arm.setSensorPhase(true);
    mc_arm.setInverted(false);
    mc_armFollower.setInverted(true);
    mc_armFollower.follow(mc_arm);

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
    // setDefaultCommand(new MySpecialCommand());
  }

  public void resetZero() { // Resets the encoder
    mc_arm.setSelectedSensorPosition(0, 0, 20);
  }

  public int getEncoderTicks() {  // Returns the ticks on the encoder
    return mc_arm.getSensorCollection().getPulseWidthPosition();
  }

  public void move(int currAngle, int dPos) { // Changes height of arm based on current angle and desired change
    mc_arm.set(ControlMode.MotionMagic, 4096 * 25 * (-currAngle+Math.acos(dPos / -length - Math.cos(currAngle))) / 360);
  }

  public void rotate(int dir) {
    if(dir < 0) {
      mc_arm.set(ControlMode.PercentOutput, 0.30 * dir); // WAS .15
      mc_wrist.set(ControlMode.PercentOutput, 0.30 * dir);
    }
    else {  
      mc_arm.set(ControlMode.PercentOutput, 0.15 * dir); // WAS .15
      mc_wrist.set(ControlMode.PercentOutput, 0.15*dir);
    }
    
  }

  public void turns(double degrees) { // Turns a certain number of degrees
    mc_arm.set(ControlMode.Position, degrees / RobotMap.ArmTicksToDeg);
    mc_wrist.set(ControlMode.Position, -degrees / RobotMap.ArmTicksToDeg);    // TODO: Get the right coefficient
    SmartDashboard.putNumber("target arm enc", degrees/RobotMap.ArmTicksToDeg);
  }

  public boolean isTurnComplete(double degrees) { // Determines if degrees of current and target match
    return (getEncoderTicks() == (degrees / RobotMap.ArmTicksToDeg));
  }
  public void moveToHeight(double height) {
    turns(Math.asin(height / RobotMap.armLength));
    
  }
  public void moveToHeightPreset(int pos) {
    if(pos < RobotMap.heights.length && pos > 0)  
      moveToHeight(RobotMap.heights[pos]);
  }
  public void intakeBall(int dir) {
    mc_intake.set(ControlMode.PercentOutput, 0.15 * dir);
  }
  public void rotateWrist(int dir) {
    mc_wrist.set(ControlMode.PercentOutput, 0.15 * dir);
  }
}
