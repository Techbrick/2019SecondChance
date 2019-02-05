/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class AccelerometerSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

    private Accelerometer accel;
    private double accX;
    private double accY;
    private double accZ;
    private Robot _robort;
    public AccelerometerSubsystem(Robot r) {
      accel = new ADXL345_I2C(Port.kOnboard, Accelerometer.Range.k4G); // change later
      _robort = r;
    }

    public double getX() 
    {
      return accel.getX();
    }
    public double getY() 
    {
      return accel.getY();
    }
    public double getZ() 
    {
      return accel.getZ();
    }
    public double getAngle() 
    {
      return Math.atan(-getX()/getY()); //Yeah i think this is how it works with the current orientation
    }
    public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
