// package frc.robot.subsystems;

// import java.util.function.Supplier;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Robot;

// /**
//  * An example subsystem.  You can replace me with your own Subsystem.
//  */
// public class DriveSubsystem extends Subsystem {
//   // Put methods for controlling this subsystem
//   // here. Call these from Commands.
//   private TalonSRX _leftMaster;
//   private TalonSRX _leftFollower;
//   private TalonSRX _rightMaster;
//   private TalonSRX _rightFollower;
//   private Robot _robot;
//   private double encoderConstant;
//   Supplier<Double> leftEncoderPosition;
// 	Supplier<Double> leftEncoderRate;
// 	Supplier<Double> rightEncoderPosition;
//   Supplier<Double> rightEncoderRate;
//   private Solenoid shifterSolenoidIn;
//   private Solenoid shifterSolenoidOut;
//   private Solenoid lifterSolenoidIn;
//   private Solenoid lifterSolenoidOut;


//   public DriveSubsystem (Robot robot){
//     _robot = robot;
//     _leftMaster = new TalonSRX(robot.robotMap.leftMaster);
//     _leftFollower = new TalonSRX(robot.robotMap.leftFollower1);
//     _rightMaster = new TalonSRX(robot.robotMap.rightMaster);
//     _rightFollower  = new TalonSRX(robot.robotMap.rightFollower1);
//     _leftFollower.setInverted(false);
//     _leftMaster.setInverted(false);
//     _rightMaster.setInverted(false);
//     _rightFollower.setInverted(false);
//     _leftFollower.follow(_leftMaster);
//     _rightFollower.follow(_rightMaster);
//     encoderConstant = (1 / robot.robotMap.driveEncoderTicksPerInch);
//     _leftMaster.clearStickyFaults(30);
//     _rightMaster.clearStickyFaults(30);
//     _leftMaster.setNeutralMode(NeutralMode.Brake);
//     _rightMaster.setNeutralMode(NeutralMode.Brake);
//     _leftFollower.clearStickyFaults(30);
//     _rightFollower.clearStickyFaults(30);
//     _leftFollower.setNeutralMode(NeutralMode.Brake);
//     _rightFollower.setNeutralMode(NeutralMode.Brake);
//     _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0, 10);
//     _leftMaster.setSelectedSensorPosition(0, 0, 10);
//     _leftMaster.setSensorPhase(true);
// 		leftEncoderPosition = () -> _leftMaster.getSelectedSensorPosition(0) * encoderConstant;
// 		leftEncoderRate = () -> _leftMaster.getSelectedSensorVelocity(0) * encoderConstant * 0.1;
		
//     _rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    
//     _rightMaster.setSelectedSensorPosition(0, 0, 10);
// 		rightEncoderPosition = () -> _rightMaster.getSelectedSensorPosition(0) * encoderConstant;
//     rightEncoderRate = () -> _rightMaster.getSelectedSensorVelocity(0) * encoderConstant * 0.1;
    
//     shifterSolenoidIn = new Solenoid(0);
//     shifterSolenoidOut = new Solenoid(1);

//     lifterSolenoidIn = new Solenoid(6);
//     lifterSolenoidOut = new Solenoid(7);

//     setShifterSolenoid(false);
//     setLifterSolenoid(false);
//   }

//   public void Move(double leftpower, double rightpower){
//     _leftMaster.set(ControlMode.PercentOutput, leftpower);
//     _rightMaster.set(ControlMode.PercentOutput, -rightpower);
//     SmartDashboard.putString("DriveTrainStatus", "Move power: "+ Double.toString(leftpower) + ", " + Double.toString(rightpower));
//   }

//   public void ArcadeDrive(double power, double turn){
//     double turnPower = turn;
//     SmartDashboard.putNumber("dg raw power", power);
//     SmartDashboard.putNumber("dg raw twist", turn);
//     _leftMaster.set(ControlMode.PercentOutput, power + turnPower);
//     _rightMaster.set(ControlMode.PercentOutput, -power+ turnPower);
//     SmartDashboard.putString("DriveTrainStatus", "ArcadeDrive power: "+ Double.toString(power));
//     SmartDashboard.putString("DriveTrainTurn", "TurnPower: " + Double.toString(turnPower));
//   }

//   public double getRobotYaw()
//   {
//     double yaw = _robot.navX.getYaw();
//     return yaw;
//   }

//   public double GetLeftEncoderPosition(){
//     double left = leftEncoderPosition.get();
//     return left;
//   }

//   public double GetRightEncoderPosition(){
//     double right = rightEncoderPosition.get();
//     return right;
//   }

//   public double GetAverageEncoderPosition(){
//     double left = leftEncoderPosition.get();
//     double right = rightEncoderPosition.get();
//     double result = (left + right)/2;
//     return result;
//   }

//   public double GetAverageEncoderRate(){
//     double left = leftEncoderRate.get();
//     double right = rightEncoderRate.get();
//     double result = (left + right)/2;
//     return result;
//   }

//   public double GetAverageEncoderPositionRaw(){
//     double left = _leftMaster.getSelectedSensorPosition(0);
//     double right = _rightMaster.getSelectedSensorPosition(0);
//     double result = (left + right)/2;
//     return result;
//   }

//   public double GetAverageEncoderRateRaw(){
//     double left = _leftMaster.getSelectedSensorVelocity(0);
//     double right = _rightMaster.getSelectedSensorVelocity(0);
//     double result = (left + right)/2;
//     return result;
//   }

//   public void ResetEncoders(){
//     _leftMaster.setSelectedSensorPosition(0, 0, 10);
//     _rightMaster.setSelectedSensorPosition(0, 0, 10);
//   }

//   public double GetLeftOutputVoltage(){
//     return _leftMaster.getMotorOutputVoltage();
//   }

//   public double GetRightOutputVoltage(){
//     return _rightMaster.getMotorOutputVoltage();
//   }

//   public void setShifterSolenoid(boolean isOpen)
//   {
//     shifterSolenoidIn.set(isOpen);
//     shifterSolenoidOut.set(!isOpen);
//   }

//   public void setLifterSolenoid(boolean isOpen)
//   {
//     lifterSolenoidIn.set(isOpen);
//     lifterSolenoidOut.set(!isOpen);
//   }

//   @Override
//   public void initDefaultCommand() {
//     setDefaultCommand(new frc.robot.commands.ManualDrive(_robot));
//   }
// }