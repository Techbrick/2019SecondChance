/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class SensorSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private NetworkTableEntry ledMode;
  private NetworkTableEntry camMode; 
  private boolean limelightEnabled;
  private Robot robot;

  public SensorSubsystem(Robot parRobot)
  {
    robot = parRobot;
    limelightEnabled = false;
    ledMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode");
    camMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode");
    enalbeLimelight();
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void enalbeLimelight()
  {
    ledMode.setNumber(limelightEnabled ? 3 : 1);
    camMode.setNumber(limelightEnabled ? 0 : 1);
  }

  public void checkLimelight()
  {
    if(limelightEnabled && (ledMode.getDouble(1) == 1 || camMode.getDouble(1) == 1))
    {
      enalbeLimelight();
    }
    if(!limelightEnabled && (ledMode.getDouble(1) == 3 || camMode.getDouble(1) == 0))
    {
      enalbeLimelight();
    }
  }

  public boolean getLimelight()
  {
    return limelightEnabled;
  }

  public void setLimelight(boolean enable)
  {
    limelightEnabled = enable;
    enalbeLimelight();
  }
}
