/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Helpers;
import frc.robot.TurnPid;

public class VisionDriveWithLimelight extends InstantCommand {
  private Robot _robot;
  // private double targetAngle;
  private double absTargetAngle;
  private double difference;
  private double tx;
  // private Helpers helper;
  private boolean drive = true;
  private TurnPid turny;
  private double absCurrentAngle;
  private Command driveToCmd;

  public VisionDriveWithLimelight(Robot robot) {
    _robot = robot;
    requires(_robot.driveTrain);
    // targetAngle = angle;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    turny = new TurnPid(_robot);
    _robot.driveTrain.Move(0,0);
    //Figure out what target angle should be
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Difference between absolute angle and target angle, then steer to it with difference * 1.5
    // double steer = m_Controller.getX(Hand.kRight);
    // double drive = -m_Controller.getY(Hand.kLeft);
    // boolean auto = m_Controller.getAButton();
    // steer *= 0.70;
    // drive *= 0.70;
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    absTargetAngle = Helpers.ConvertYawToHeading(tx + _robot.navX.getYaw());
    absCurrentAngle = Helpers.ConvertYawToHeading(_robot.navX.getYaw());
    difference = 1.5 * (absTargetAngle - absCurrentAngle);
    // difference = absTargetAngle - targetAngle;
    _robot.driveTrain.Update_Limelight_Tracking();
    // turny.SetTargetAngle(Helpers.ConvertYawToHeading(difference*1.5 + targetAngle));
    turny.SetTargetAngle(Helpers.ConvertYawToHeading(absTargetAngle));

    SmartDashboard.putNumber("Target", absTargetAngle);
    SmartDashboard.putNumber("Current", absCurrentAngle);
    SmartDashboard.putNumber("Angle Error", difference);
    if (drive && _robot.driveTrain.m_LimelightHasValidTarget)
    {
      // This part is new:
      driveToCmd = new DriveDistanceAndDirection(_robot, NetworkTableInstance.getDefault().getTable("LIDAR").getEntry("dist").getDouble(0) * 0.039, tx);
      driveToCmd.start();
    }
    else
    {
      _robot.driveTrain.ArcadeDrive(0.0,0.0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _robot.driveTrain.Move(0,0);
  }

// Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    _robot.driveTrain.Move(0,0);
  }
}
