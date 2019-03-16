/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.SensorPet;
import frc.robot.subsystems.SensorSubsystem;

// import java.util.function.Supplier;
// import com.ctre.phoenix.*;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

// import org.omg.CORBA.PRIVATE_MEMBER;
// import org.opencv.core.Mat;
// import org.opencv.imgproc.Imgproc;

// import edu.wpi.cscore.CvSink;
// import edu.wpi.cscore.CvSource;
// import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  public static OI m_oi;
  public RobotMap robotMap = new RobotMap();
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  public NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  public NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

  public Joystick DrvStick;
  public Joystick operatorStick;
	public double encoderConstant;
	
  public DriveSubsystem driveTrain;
  public ArmSubsystem arm_subsystem;
  public CompressorSubsystem comp_subsystem;
  public SensorSubsystem sensor_subsystem;
  public AHRS navX;
  public AHRS wristnavX;
  double priorAutospeed = 0;
	Number[] numberArray = new Number[9];
  public DigitalInput DI = new DigitalInput(1);
  // public SensorPet pet = new SensorPet();

  public Spark MC_LEDS = new Spark(0);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    SmartDashboard.putString("Instructions", "");
    SmartDashboard.putString("Status", "");
    DrvStick = new Joystick(0);
    operatorStick= new Joystick(1);
    navX = new AHRS(SPI.Port.kMXP);
    wristnavX = new AHRS(Port.kUSB);

    driveTrain = new DriveSubsystem(this);
    arm_subsystem = new ArmSubsystem(this);
    comp_subsystem = new CompressorSubsystem(this);
    sensor_subsystem = new SensorSubsystem(this);
    
    MC_LEDS.setSafetyEnabled(false);
    MC_LEDS.setSpeed(-0.99);
    
    SmartDashboard.putData(driveTrain);
    SmartDashboard.putData(arm_subsystem);
    SmartDashboard.putData("Drive Encoder Cal", new DriveEncoderCal(this));
    SmartDashboard.putData("Manual Drive", new ManualDrive(this));
    SmartDashboard.putData("Min Turn Power", new FindMinTurnPower(this));
    SmartDashboard.putData("Min Drive Power", new FindMinDrivePower(this));
    SmartDashboard.putData("Tune Turn Pid", new TuneTurnPid(this));
    SmartDashboard.putData("Tune Distance Pid", new TuneDistancePid(this));
    SmartDashboard.putData("Turn left 90", new TestTurnLeft90(this));
    SmartDashboard.putData("Test Turn Right 90", new TestTurnRight90(this));
    SmartDashboard.putData("Test Fwd 48", new TestMoveFwd48(this));
    SmartDashboard.putData("Test back 48", new TestMoveBack48(this));
    SmartDashboard.putData("Manual Arm", new ManualArm(this));
    SmartDashboard.putData("Dog Left", new DogLegLeft(this) );
    SmartDashboard.putData("Rotate 1", new Turn(this, 1));
    SmartDashboard.putData("LIDAR assisted auto-place", new VisionDriveWithLimelight(this));

    // SmartDashboard.putData("Height 0", new MoveToHeight(this, 0));
    // SmartDashboard.putData("Height 1", new MoveToHeight(this, 1));
    // SmartDashboard.putData("Height 2", new MoveToHeight(this, 2));
    // SmartDashboard.putData("Height 3", new MoveToHeight(this, 3));
    // SmartDashboard.putData("Height 4", new MoveToHeight(this, 4));
    // SmartDashboard.putData("Height 5", new MoveToHeight(this, 5));
    // SmartDashboard.putData("Height 6", new MoveToHeight(this, 6));
    // SmartDashboard.putData("Height 7", new MoveToHeight(this, 7));
    // SmartDashboard.putData("Height 8", new MoveToHeight(this, 8));
    // SmartDashboard.putData("Stowreset", new ResetAutoArm(this));

    SmartDashboard.putData("RocketAngle", new VisionDrive(this)); // You realize that there are 2 rockets?
    SmartDashboard.putData("Straight", new VisionDrive(this));
    SmartDashboard.putData("RocketAngleBackSide", new VisionDrive(this));
    SmartDashboard.putData("Left", new VisionDrive(this));
    SmartDashboard.putData("Right", new VisionDrive(this));

    // m_chooser.addObject("Drive Fwd 24 inches", new DriveDistanceAndDirection(this, 24, 0));
    // m_chooser.addObject("Drive dog leg right", new DogLegRight(this));
    // m_chooser.addObject("Drive dog leg left", new DogLegLeft(this));
    // SmartDashboard.putData("Auto mode", m_chooser);
		
    NetworkTableInstance.getDefault().setUpdateRate(0.020);
    
    m_oi = new OI(this);
    navX.zeroYaw();
    CameraServer.getInstance().startAutomaticCapture();
   }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Logger();
    sensor_subsystem.checkLimelight();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    driveTrain.Move(0,0);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    Logger();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    // double power =  DrvStick.getY();
    // double twist = DrvStick.getX();
    //driveTrain.ArcadeDrive(power, twist);
    Logger();
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testInit() {
    super.testInit();
    // 
    robotMap.verbose = true;
  }

  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
    Logger();
  }
  //   this function is needed for commands to read position;
  
  private void Logger(){
    if(robotMap.verbose){
      SmartDashboard.putNumber("l_encoder_pos", Math.round(driveTrain.GetLeftEncoderPosition()));
      // SmartDashboard.putNumber("l_encoder_rate", Math.round(leftEncoderRate.get()));
      SmartDashboard.putNumber("r_encoder_pos", Math.round(driveTrain.GetRightEncoderPosition()));
      // SmartDashboard.putNumber("r_encoder_rate", Math.round(rightEncoderRate.get()));
      SmartDashboard.putNumber("navX Heading", navX.getCompassHeading());
      SmartDashboard.putNumber("navX Angle", Math.round(navX.getRawMagX()));
      SmartDashboard.putNumber("navX yaw", navX.getYaw());
      SmartDashboard.putNumber("avgEncoderRate", driveTrain.GetAverageEncoderRate());
      SmartDashboard.putNumber("Arm Encoder Ticks", arm_subsystem.getArmEncoderTicks()); //Ticks * bleh turns ticks into angles, / 25 to get past the reduction. 
      SmartDashboard.putNumber("Arm Angle", arm_subsystem.getArmEncoderAngle());
      SmartDashboard.putNumber("Wrist Encoder Ticks", arm_subsystem.getWristEncoderTicks());
      SmartDashboard.putNumber("Wrist Encoder Angle", arm_subsystem.getWristEncoderTicks() * 360 / (4096 * 25));
      SmartDashboard.putBoolean("navXConnected", navX.isConnected());
      SmartDashboard.putBoolean("navXConnected", wristnavX.isConnected());
      SmartDashboard.putNumber("QuaternionW", wristnavX.getQuaternionW());
      SmartDashboard.putNumber("QuaternionX", wristnavX.getQuaternionX());
      SmartDashboard.putNumber("QuaternionY", wristnavX.getQuaternionY());
      SmartDashboard.putNumber("QuaternionZ", wristnavX.getQuaternionZ());
      SmartDashboard.putNumber("Wisty Angle", arm_subsystem.getWistAngle());
      SmartDashboard.putNumber("WristStartAngle", arm_subsystem.wristStartAngle);
      SmartDashboard.putNumber("Drivetrain encoder rate", driveTrain.GetAverageEncoderRate()*12);
    }
    
    // SmartDashboard.putBoolean("bit1", pet.getbit1());
    // SmartDashboard.putBoolean("bit2", pet.getbit2());
    // SmartDashboard.putBoolean("bit3", pet.getbit3());
    // SmartDashboard.putBoolean("bit4", pet.getbit4());
    
    SmartDashboard.putNumber("Navx Roll", navX.getRoll());
    SmartDashboard.putNumber("Navx Pitch", navX.getPitch());
    SmartDashboard.putNumber("Navx Yaw", navX.getYaw());

    //IMPORTANT BOOLEANS
    SmartDashboard.putBoolean("Hatch/Ball toggle", arm_subsystem.getToggly());
    SmartDashboard.putBoolean("HatchEjector", arm_subsystem.getHatchEjectorValue());
    SmartDashboard.putBoolean("Ball in", DI.get());
  }
}
