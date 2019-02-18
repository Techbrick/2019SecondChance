/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Helpers;
public class VisionDrive extends Command {
  Robot _robot;
  double targetAngle;
  double absoluteAngle;
  double difference;
  double tx;
  Helpers helper;
  boolean drive = true;
  public VisionDrive(Robot robot, int angle) {
    _robot = robot;
    requires(robot.driveTrain);
    targetAngle = angle;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
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
        absoluteAngle = tx + helper.ConvertYawToHeading(_robot.navX.getYaw());
        difference = absoluteAngle - targetAngle;
        _robot.driveTrain.Update_Limelight_Tracking();
        if (drive)
        {
          if (_robot.driveTrain.m_LimelightHasValidTarget)
          {
            if(difference<25)
            {
              _robot.driveTrain.ArcadeDrive(-_robot.driveTrain.m_LimelightDriveCommand,difference*1.5*.03); //_robot.driveTrain.m_LimelightSteerCommand
            }
            else
            {
              _robot.driveTrain.ArcadeDrive(-_robot.driveTrain.m_LimelightDriveCommand,25*0.03);
            }
          }
          else
          {
                _robot.driveTrain.ArcadeDrive(0.0,0.0);
          }
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
  }

// Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
