/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.WristPid;

public class ManualArm1 extends Command {
  private Robot _robot;
  private WristPid wristy;
  private int ArmUpperOrangeLimit = 14000;
  private int ArmUpperRedLimit = 12000;
  private int ArmLowerRedLimit = 9500;
  private int ArmLowerOrangeLimit = 8500;
  private double WristHatchAngle = -20;
  private double WristFlat = -60;
  private double WristBreakover = -55;

  public ManualArm1(Robot r) {
   
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
    double wristCurrentPosition = _robot.arm_subsystem.getWistAngle();

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
    if(armOrange){
      if(!(wristGreen || wristInBall)){
        if(wristCloserToHatch){
          wristy.SetTargetAngle(WristHatchAngle);
          double wristTurn = wristy.GetAnglePidOutput(wristCurrentPosition);
          _robot.arm_subsystem.setWristSpeed(wristTurn);//2
        }else{
          wristy.SetTargetAngle(WristFlat);
          double wristTurn = wristy.GetAnglePidOutput(wristCurrentPosition);
          _robot.arm_subsystem.setWristSpeed(wristTurn);//2
        }
        if(!armRed){
          _robot.arm_subsystem.setArmSpeed(-_robot.operatorStick.getRawAxis(5)*.5);//0
        }
      }
      else{
        _robot.arm_subsystem.setArmSpeed(-_robot.operatorStick.getRawAxis(5));//0
      }
    }else{
      _robot.arm_subsystem.setWristSpeed(_robot.operatorStick.getRawAxis(1));//2
      _robot.arm_subsystem.setArmSpeed(-_robot.operatorStick.getRawAxis(5));//0

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
    _robot.arm_subsystem.rotate(0);
    _robot.arm_subsystem.rotateWrist(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
