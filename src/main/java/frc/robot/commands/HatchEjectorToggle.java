/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;

public class HatchEjectorToggle extends ConditionalCommand  {
  private Robot robot;
  public HatchEjectorToggle(Command parOnTrue, Command parOnFalse, Robot parRobot)
  {
     super(parOnTrue, parOnFalse);
     robot = parRobot;
  }
  
  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected boolean condition() {
    return robot.arm_subsystem.getHatchEjectorValue() == true;
  }
}
