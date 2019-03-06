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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

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
  private DoubleSolenoid shifterSolenoid;
  private DoubleSolenoid lifterSolenoid;
  public boolean m_LimelightHasValidTarget;
  public double m_LimelightDriveCommand;
  public double m_LimelightSteerCommand;

  public RealDriveTrain(Robot robot) {
    _robot = robot;
    _leftMaster = new TalonSRX(RobotMap.leftMaster);
    _leftFollower1 = new VictorSPX(RobotMap.leftFollower1);
    _leftFollower2 = new VictorSPX(RobotMap.leftFollower2);
    _rightMaster = new TalonSRX(RobotMap.rightMaster);
    _rightFollower1 = new VictorSPX(RobotMap.rightFollower1);
    _rightFollower2 = new VictorSPX(RobotMap.rightFollower2);
    _leftFollower1.setInverted(false);
    _leftFollower2.setInverted(false);
    _leftMaster.setInverted(false);
    _rightMaster.setInverted(false);
    _rightFollower1.setInverted(false);
    _rightFollower2.setInverted(false);
    _leftFollower1.follow(_leftMaster);
    _leftFollower2.follow(_leftMaster);
    _rightFollower1.follow(_rightMaster);
    _rightFollower2.follow(_rightMaster);
    encoderConstant = (1 / RobotMap.driveEncoderTicksPerInch);
    _leftMaster.clearStickyFaults(30);
    _rightMaster.clearStickyFaults(30);
    _leftFollower1.clearStickyFaults(30);
    _leftFollower2.clearStickyFaults(30);
    _rightFollower1.clearStickyFaults(30);
    _rightFollower2.clearStickyFaults(30);
    _leftMaster.setNeutralMode(NeutralMode.Coast);
    _rightMaster.setNeutralMode(NeutralMode.Coast);
    _leftFollower1.setNeutralMode(NeutralMode.Coast);
    _leftFollower2.setNeutralMode(NeutralMode.Coast);
    _rightFollower1.setNeutralMode(NeutralMode.Coast);
    _rightFollower2.setNeutralMode(NeutralMode.Coast);
    _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0, 10);
    _leftMaster.setSelectedSensorPosition(0, 0, 10);
    _leftMaster.setSensorPhase(true);
		leftEncoderPosition = () -> _leftMaster.getSelectedSensorPosition(0) * encoderConstant;
		leftEncoderRate = () -> _leftMaster.getSelectedSensorVelocity(0) * encoderConstant * 0.1;
		
    _rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    _rightMaster.setSelectedSensorPosition(0, 0, 10);
		rightEncoderPosition = () -> _rightMaster.getSelectedSensorPosition(0) * encoderConstant;
    rightEncoderRate = () -> _rightMaster.getSelectedSensorVelocity(0) * encoderConstant * 0.1;
    
    _leftMaster.configMotionAcceleration(100);
    _leftFollower1.configMotionAcceleration(100);
    _leftFollower2.configMotionAcceleration(100);
    _rightMaster.configMotionAcceleration(100);
    _rightFollower1.configMotionAcceleration(100);
    _rightFollower2.configMotionAcceleration(100);

    shifterSolenoid = new DoubleSolenoid(0,1);
    shifterSolenoid.set(Value.kOff);

    lifterSolenoid = new DoubleSolenoid(6,7);
    lifterSolenoid.set(Value.kOff);
  }

  public void Move(double leftpower, double rightpower){
    _leftMaster.set(ControlMode.PercentOutput, leftpower);
    _rightMaster.set(ControlMode.PercentOutput, rightpower);
    SmartDashboard.putString("DriveTrainStatus", "Move power: "+ Double.toString(leftpower) + ", " + Double.toString(rightpower));
  }
  public void ArcadeDrive(double power, double turn){
    // double turnPower = turn;
    // SmartDashboard.putNumber("dg raw power", power);
    // SmartDashboard.putNumber("dg raw twist", turn);
    // _leftMaster.set(ControlMode.PercentOutput, manageDeadband(-power - turnPower));
    // _rightMaster.set(ControlMode.PercentOutput, manageDeadband(power - turnPower));
    // // SmartDashboard.putNumbers
    // SmartDashboard.putString("DriveTrainStatus", "ArcadeDrive power: "+ Double.toString(power));
    // SmartDashboard.putString("DriveTrainTurn", "TurnPower: " + Double.toString(turnPower));
    SmartDashboard.putNumber("Raw power", power);
    SmartDashboard.putNumber("Raw turn", turn);
    double x = -power;
    double y = -turn;
    double v = (100.0 - Math.abs(x)) * (y / 100.0) + y;
    double w = (100.0 - Math.abs(y)) * (x / 100.0) + x;
    double r = (v + w) / 2.0;
    double l = (v-w) / 2.0;

    //Yo dudes theres a double deadband
    _leftMaster.set(ControlMode.PercentOutput, l);
    _rightMaster.set(ControlMode.PercentOutput, r);
    SmartDashboard.putNumber("Left Input", l);
    SmartDashboard.putNumber("Right Input", r);
  }

  public double manageDeadband(double power) {
    double adjustedPower = power;
    if(Math.abs(adjustedPower) <= 0.1)
      adjustedPower = 0;
    return Math.pow(adjustedPower, 3);
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

  public void setShifterSolenoid(boolean isOpen)
  {
    shifterSolenoid.set(isOpen ? Value.kForward : Value.kReverse);
  }

  public void setLifterSolenoid(boolean isOpen)
  {
    lifterSolenoid.set(isOpen ? Value.kForward : Value.kReverse);
  }

  public void setBrakeMode(boolean brake){
    if(brake){
      _leftMaster.setNeutralMode(NeutralMode.Brake);
      _rightMaster.setNeutralMode(NeutralMode.Brake);
      _leftFollower1.setNeutralMode(NeutralMode.Brake);
      _leftFollower2.setNeutralMode(NeutralMode.Brake);
      _rightFollower1.setNeutralMode(NeutralMode.Brake);
      _rightFollower2.setNeutralMode(NeutralMode.Brake);
    }
    else{
      _leftMaster.setNeutralMode(NeutralMode.Coast);
      _rightMaster.setNeutralMode(NeutralMode.Coast);
      _leftFollower1.setNeutralMode(NeutralMode.Coast);
      _leftFollower2.setNeutralMode(NeutralMode.Coast);
      _rightFollower1.setNeutralMode(NeutralMode.Coast);
      _rightFollower2.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 1;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 2.3;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
        final double minDrive = 0.2;
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);


    if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;
        SmartDashboard.putBoolean("LimelightTarget", m_LimelightHasValidTarget);

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;
        SmartDashboard.putNumber("ll steer", m_LimelightSteerCommand);

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        if(drive_cmd < minDrive)
        {
          drive_cmd = minDrive;
        }
        
        m_LimelightDriveCommand = drive_cmd;
        SmartDashboard.putNumber("LL DRIVE", m_LimelightDriveCommand);
  }
  @Override
  public void initDefaultCommand() {
  
    setDefaultCommand(new frc.robot.commands.ManualDrive(_robot));
  }
}
 