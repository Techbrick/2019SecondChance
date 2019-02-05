/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class RealDriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Robot _robot;
  private TalonSRX _leftMaster;
  private TalonSRX _rightMaster;
  private VictorSPX _leftFollower1;
  private VictorSPX _leftFollower2;
  private VictorSPX _rightFollower1;
  private VictorSPX _rightFollower2;
  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  private double encoderConstant;

  public RealDriveTrain(Robot robot) {
    _robot = robot;
    _leftMaster = new TalonSRX(robot.robotMap.leftMaster);
    _leftFollower1 = new VictorSPX(robot.robotMap.leftFollower1);
    _leftFollower2 = new VictorSPX(robot.robotMap.leftFollower2);
    _rightMaster = new TalonSRX(robot.robotMap.rightMaster);
    _rightFollower1 = new VictorSPX(robot.robotMap.rightFollower1);
    _rightFollower2 = new VictorSPX(robot.robotMap.rightFollower2);
    _leftFollower1.setInverted(true);
    _leftFollower2.setInverted(true);
    _leftMaster.setInverted(true);
    _rightMaster.setInverted(false);
    _rightFollower1.setInverted(false);
    _rightFollower2.setInverted(false);
    _leftFollower1.follow(_leftMaster);
    _leftFollower2.follow(_leftMaster);
    _rightFollower1.follow(_rightMaster);
    _rightFollower2.follow(_rightMaster);
    encoderConstant = (1 / robot.robotMap.driveEncoderTicksPerInch);
    _leftMaster.clearStickyFaults(30);
    _rightMaster.clearStickyFaults(30);
    _leftMaster.setNeutralMode(NeutralMode.Brake);
    _rightMaster.setNeutralMode(NeutralMode.Brake);
    _leftFollower1.clearStickyFaults(30);
    _leftFollower2.clearStickyFaults(30);
    _rightFollower1.clearStickyFaults(30);
    _rightFollower2.clearStickyFaults(30);
    _leftFollower1.setNeutralMode(NeutralMode.Brake);
    _leftFollower2.setNeutralMode(NeutralMode.Brake);
    _rightFollower1.setNeutralMode(NeutralMode.Brake);
    _rightFollower2.setNeutralMode(NeutralMode.Brake);
    _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0, 10);
    _leftMaster.setSelectedSensorPosition(0, 0, 10);
    _leftMaster.setSensorPhase(true);
		leftEncoderPosition = () -> _leftMaster.getSelectedSensorPosition(0) * encoderConstant;
		leftEncoderRate = () -> _leftMaster.getSelectedSensorVelocity(0) * encoderConstant * 0.1;
		
    _rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    
    _rightMaster.setSelectedSensorPosition(0, 0, 10);
		rightEncoderPosition = () -> _rightMaster.getSelectedSensorPosition(0) * encoderConstant;
		rightEncoderRate = () -> _rightMaster.getSelectedSensorVelocity(0) * encoderConstant * 0.1;
  }

  public void Move(double leftpower, double rightpower){
    _leftMaster.set(ControlMode.PercentOutput, leftpower);
    _rightMaster.set(ControlMode.PercentOutput, -rightpower);
    SmartDashboard.putString("DriveTrainStatus", "Move power: "+ Double.toString(leftpower) + ", " + Double.toString(rightpower));
  }
  public void ArcadeDrive(double power, double turn){
    double turnPower = turn;
    SmartDashboard.putNumber("dg raw power", power);
    SmartDashboard.putNumber("dg raw twist", turn);
    _leftMaster.set(ControlMode.PercentOutput, manageDeadband(power + turnPower));
    _rightMaster.set(ControlMode.PercentOutput, manageDeadband(-power + turnPower));
    SmartDashboard.putString("DriveTrainStatus", "ArcadeDrive power: "+ Double.toString(power));
    SmartDashboard.putString("DriveTrainTurn", "TurnPower: " + Double.toString(turnPower));
  }

  public double manageDeadband(double power) {
    double adjustedPower = Math.pow(power, 3);
    if(adjustedPower <= 0.05)
      return 0;
    else
      return adjustedPower;
  }
  public double getRobotYaw()
  {
    double yaw = _robot.navX.getYaw();
    return yaw;
  }
  public double GetLeftEncoderPosition(){
    double left = leftEncoderPosition.get();
    return left;
  }
  public double GetRightEncoderPosition(){
    double right = rightEncoderPosition.get();
    return right;
  }
  public double GetAverageEncoderPosition(){
    double left = leftEncoderPosition.get();
    double right = rightEncoderPosition.get();
    double result = (left + right)/2;
    return result;

  }
  public double GetAverageEncoderRate(){
    double left = leftEncoderRate.get();
    double right = rightEncoderRate.get();
    double result = (left + right)/2;
    return result;

  }
  public double GetAverageEncoderPositionRaw(){
    double left = _leftMaster.getSelectedSensorPosition(0);
    double right = _rightMaster.getSelectedSensorPosition(0);
    double result = (left + right)/2;
    return result;

  }
  public double GetAverageEncoderRateRaw(){
    double left = _leftMaster.getSelectedSensorVelocity(0);
    double right = _rightMaster.getSelectedSensorVelocity(0);
    double result = (left + right)/2;
    return result;

  }
  public void ResetEncoders(){
    _leftMaster.setSelectedSensorPosition(0, 0, 10);
    _rightMaster.setSelectedSensorPosition(0, 0, 10);
  }
  public double GetLeftOutputVoltage(){
    return _leftMaster.getMotorOutputVoltage();

  }
  public double GetRightOutputVoltage(){
    return _rightMaster.getMotorOutputVoltage();

  }
  @Override
  public void initDefaultCommand() {
  
    setDefaultCommand(new frc.robot.commands.ManualDrive(_robot));
  }
}
 