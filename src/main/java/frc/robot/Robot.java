/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
// import frc.robot.subsystems.AccelerometerSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RealDriveTrain;
import frc.robot.subsystems.SensorPet;

import java.util.function.Supplier;
import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.Quaternion;

//import org.omg.CORBA.PRIVATE_MEMBER;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
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
  public SensorPet pet = new SensorPet();
  
  public 
  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  
  public Joystick DrvStick;
  public Joystick operatorStick;
	public double encoderConstant;
	
  // public TalonSRX leftMaster;
  // private TalonSRX leftFollower;
  // public TalonSRX rightMaster;
  // private TalonSRX rightFollower;
  // public  DriveSubsystem driveTrain;
  public RealDriveTrain driveTrain;
  public ArmSubsystem arm_subsystem;
  public CompressorSubsystem comp_subsystem;
  // public AccelerometerSubsystem accelerometer_subsystem;
  public AHRS navX;
  public AHRS wristnavX;
  double priorAutospeed = 0;
	Number[] numberArray = new Number[9];
  public DigitalInput DI = new DigitalInput(1);
  private Helpers helper;
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
    robotMap.verbose = true;

    navX = new AHRS(SPI.Port.kMXP);
    wristnavX = new AHRS(Port.kUSB);
    
    driveTrain = new RealDriveTrain(this);
    arm_subsystem = new ArmSubsystem(this);
    comp_subsystem = new CompressorSubsystem(this);
    helper = new Helpers();
    // accelerometer_subsystem = new AccelerometerSubsystem(this);
    
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
    //SmartDashboard.putData("DriveAlign", new DriveAlign(this));

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
    // SmartDashboard.putData("Accelerometer Angle", new AccelerometerAngle(this));

    SmartDashboard.putData("RocketAngle", new VisionDrive(this,-60));
    SmartDashboard.putData("Straight", new VisionDrive(this,0));
    SmartDashboard.putData("RocketAngleBackSide", new VisionDrive(this,30));
    SmartDashboard.putData("Left", new VisionDrive(this,-90));
    SmartDashboard.putData("Right", new VisionDrive(this,90));

    // Shuffleboard.getTab("Camera").add("Compression slider", );
    // m_chooser.addObject("Drive Fwd 24 inches", new DriveDistanceAndDirection(this, 24, 0));
    // m_chooser.addObject("Drive dog leg right", new DogLegRight(this));
    // m_chooser.addObject("Drive dog leg left", new DogLegLeft(this));
    // SmartDashboard.putData("Auto mode", m_chooser);
		
    NetworkTableInstance.getDefault().setUpdateRate(0.020);
    
    m_oi = new OI(this);
    double initYaw = navX.getYaw();
    SmartDashboard.putNumber("intYaw", initYaw);
    //navX.reset();
    navX.zeroYaw();
    // new Thread(() -> {
    //   UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    //   camera.setResolution(640, 480);
      
    //   CvSink cvSink = CameraServer.getInstance().getVideo();
    //   CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
      
    //   Mat source = new Mat();
    //   Mat output = new Mat();
      
    //   while(!Thread.interrupted()) {
    //       cvSink.grabFrame(source);
    //       Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
    //       outputStream.putFrame(output);
    //   }
  // }).start();

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
    if(true){
      SmartDashboard.putNumber("l_encoder_pos", Math.round(driveTrain.GetLeftEncoderPosition()));
      // SmartDashboard.putNumber("l_encoder_rate", Math.round(leftEncoderRate.get()));
      SmartDashboard.putNumber("r_encoder_pos", Math.round(driveTrain.GetRightEncoderPosition()));
      // SmartDashboard.putNumber("r_encoder_rate", Math.round(rightEncoderRate.get()));
      SmartDashboard.putNumber("navx pitch", Math.round(navX.getPitch()));
      SmartDashboard.putNumber("navx Heading", navX.getCompassHeading());
      SmartDashboard.putNumber("navx Angle", Math.round(navX.getRawMagX()));
      SmartDashboard.putNumber("avgEncoderRate", driveTrain.GetAverageEncoderRate());
      SmartDashboard.putNumber("Arm Encoder Ticks", arm_subsystem.getArmEncoderTicks());
      //Ticks * bleh turns ticks into angles, / 25 to get past the reduction. 
      SmartDashboard.putNumber("Arm Encoder Angle", arm_subsystem.getArmEncoderTicks() * 360 / (4096 * 25));
      SmartDashboard.putNumber("Wrist Encoder Ticks", arm_subsystem.getWristEncoderTicks());

      SmartDashboard.putNumber("Wrist Encoder Angle", arm_subsystem.getWristEncoderTicks() * 360 / (4096 * 25));
      SmartDashboard.putBoolean("Ball in", DI.get());

      SmartDashboard.putNumber("raw yaw", wristnavX.getYaw());
      SmartDashboard.putNumber("raw pitch", wristnavX.getPitch());
      SmartDashboard.putNumber("raw roll", wristnavX.getRoll());
      SmartDashboard.putBoolean("HatchEjector", arm_subsystem.getHatchEjectorValue());
    }
    
    SmartDashboard.putBoolean("navXConnected", navX.isConnected());
    // SmartDashboard.putNumber("navX yaw", Math.round(driveTrain.getRobotYaw()));
    // SmartDashboard.putNumber("Raw yaw", Math.round(yaw));
    //SmartDashboard.putBoolean("joystick buttom", stick.getRawButton(1));
    double fps = driveTrain.GetAverageEncoderRate()*12;
    SmartDashboard.putNumber("Drivetrain encoder rate", fps);
    SmartDashboard.putBoolean("bit1", pet.getbit1());
    SmartDashboard.putBoolean("bit2", pet.getbit2());
    SmartDashboard.putBoolean("bit3", pet.getbit3());
    SmartDashboard.putBoolean("bit4", pet.getbit4());
    SmartDashboard.putBoolean("Hatch/Ball toggle", arm_subsystem.getToggly());
    
    SmartDashboard.putNumber("QuaternionW", wristnavX.getQuaternionW());
    SmartDashboard.putNumber("QuaternionX", wristnavX.getQuaternionX());
    SmartDashboard.putNumber("QuaternionY", wristnavX.getQuaternionY());
    SmartDashboard.putNumber("QuaternionZ", wristnavX.getQuaternionZ());
    SmartDashboard.putNumber("Quaternion Angle", Math.toDegrees(Math.atan2(wristnavX.getQuaternionY(), wristnavX.getQuaternionW())));
    SmartDashboard.putNumber("WristStartAngle", arm_subsystem.wristStartAngle);
  }

}
