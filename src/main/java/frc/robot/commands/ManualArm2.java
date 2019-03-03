/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Helpers;
import frc.robot.Robot;
import frc.robot.WristPid;

public class ManualArm2 extends Command {
  private Robot _robot;
  private WristPid wristy;
  private boolean on = false;
  private int ArmUpperOrangeLimit = 18000;
  private int ArmUpperRedLimit = 17000;
  private int ArmLowerRedLimit = 15000;
  private int ArmLowerOrangeLimit = 14000;
  private double WristHatchAngle = -30;
  private double WristFlat = -65;
  private double WristBreakover = -50;

  public ManualArm2(Robot r) {
   
    _robot = r;
    requires(_robot.arm_subsystem);
    wristy = new WristPid(.02, 0,0,.1, .02, 1, 1.0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double wristCurrentPosition = Math.toDegrees(Math.atan2(_robot.wristnavX.getQuaternionY(), _robot.wristnavX.getQuaternionW()));
    float armMultiplier = 1;
    int armCurrentPosition = _robot.arm_subsystem.getArmEncoderTicks();
    boolean armOrange = armCurrentPosition > ArmLowerOrangeLimit && armCurrentPosition < ArmUpperOrangeLimit;
    boolean armRed = armCurrentPosition > ArmLowerRedLimit & armCurrentPosition < ArmUpperRedLimit;
    boolean wristGreen = wristCurrentPosition > (WristHatchAngle - 5.0);
    boolean wristInBall = wristCurrentPosition > (WristFlat - 5.0) && wristCurrentPosition < (WristFlat + 5.0);
    boolean wristCloserToHatch = wristCurrentPosition > WristBreakover;

    SmartDashboard.putBoolean("armOrange", armOrange);
    SmartDashboard.putBoolean("armRed", armRed);
    SmartDashboard.putBoolean("wristGreen", wristGreen);
    SmartDashboard.putBoolean("wristInBall", wristInBall);
    if(armOrange)
    {
      armMultiplier = armRed && !wristGreen ? 0 : 0.5F;
      wristy.SetTargetAngle(wristCloserToHatch ? WristHatchAngle : WristFlat);
      _robot.arm_subsystem.setWristSpeed(wristy.GetAnglePidOutput(wristy.getCurrentAngle())); 
    }
    else
    {
      _robot.arm_subsystem.setWristSpeed(_robot.operatorStick.getRawAxis(1));
    }

    _robot.arm_subsystem.setArmSpeed(-_robot.operatorStick.getRawAxis(5) * armMultiplier);



  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _robot.arm_subsystem.rotate(0);
    _robot.arm_subsystem.rotateWrist(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
