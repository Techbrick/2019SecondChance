package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristPid {

    private double _kp;
    private double _ki;
    private double _kd;
    private double _interval;
    private double _minTurnPower;
    private double _lastError;
    private double _accumulatedI;
    private boolean start;
    private double _deadband;
    private double _targetAngle;
    private boolean _verbose;
    private Robot _robot;
    private double _maxPidPower;

    public WristPid(double kp, double ki, double kd, double minTurnPower, double interval, double deadband, double maxTurnPower){
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _minTurnPower = minTurnPower;
        _interval = interval;
        _accumulatedI = 0;
        _deadband = deadband;
        _maxPidPower = maxTurnPower;
        _verbose = false;
        start = true;
    }

    public WristPid(Robot robot){
        _robot = robot;
        _kp = _robot.robotMap.kp_Angle_Wrist;
        _ki = _robot.robotMap.ki_Angle_Wrist;
        _kd = _robot.robotMap.kd_Angle_Wrist;
        _minTurnPower = _robot.robotMap.minWristPower;
        _interval = _robot.robotMap.timingInterval;
        _accumulatedI = 0;
        _deadband = _robot.robotMap.pidWristDeadband;
        _verbose = _robot.robotMap.verbose;
        _maxPidPower = _robot.robotMap.maxPidPower;
        start = true;
    }

    public void SetTargetAngle(double targetAngle){
        _targetAngle = targetAngle;
    }

    public double getTargetAngle(){
        return _targetAngle;
    }

    public double getCurrentAngle(){
        //Dans dumb not degree angleydo.
        double dansAngle = Math.toDegrees(Math.atan2(_robot.wristnavX.getQuaternionY(), _robot.wristnavX.getQuaternionW()));
        //if in normal mode, do nothing special.
            //dansAngle -= _robot.arm_subsystem.wristStartAngle;
            if(_robot.arm_subsystem.wristStartAngle < 0)
            {
                dansAngle -= _robot.arm_subsystem.wristStartAngle * ( dansAngle < 0 ? 1 : -1);
            }
            else
            {
                dansAngle -= _robot.arm_subsystem.wristStartAngle;
            }
            SmartDashboard.putNumber("The pipes are calling", dansAngle);
        return dansAngle;
    }

    public double GetAnglePidOutput(double currentAngle) {
        //currentAngle = Helpers.ConvertYawToHeading(currentAngle);
        if(start){
            SmartDashboard.putString("WPid t Status", "Started New PidWrist Class");
        }
        double angle_error = getTargetAngle() - currentAngle; //calculate error
        if(angle_error > 180){
            angle_error = 360-angle_error;
        } else if(angle_error < -180){
            angle_error = angle_error + 360;
        }
        //angle_error = Math.abs(angle_error) > 180 ? 180 - angle_error : angle_error; //scale error to take shortest path
        // if (_targetAngle == 0 && currentAngle > 180) {
        //         angle_error = currentAngle - 360;
        // }
        double p_Angle = _kp * angle_error; //calculate p
        _accumulatedI += _ki * (angle_error * _interval); //calculate i
        double i_Angle = _ki*_accumulatedI;
        double d_Angle = 0;
        if (!start){
            d_Angle = _kd * ((angle_error - _lastError) / _interval); //calculate d
        }
        start = false;
        
        double angleOutput = p_Angle + i_Angle + d_Angle; //calculate output
        _lastError = angle_error; //set last angle error for d value
      
        angleOutput = Math.abs(angleOutput) < _minTurnPower ? Math.copySign(_minTurnPower, angleOutput) : angleOutput; //if angleOutput is below min, set to min
        angleOutput = Math.abs(angleOutput) > _maxPidPower ? Math.copySign(_maxPidPower, angleOutput) : angleOutput; //if angleOutput is above max, set to max
        //angleOutput = angle_error < 0 ? angleOutput : -angleOutput;
        if (Math.abs(angle_error) < _deadband) { //if done moving
            i_Angle = 0;
            angleOutput = 0;
            SmartDashboard.putString("Pid t Status", "PidTurn Class completed");
        }
        //angleOutput = -angleOutput;
        if(_verbose){
            SmartDashboard.putNumber("TEST angle pwr ", angleOutput);
            SmartDashboard.putNumber("WTEST angle error", angle_error);
            SmartDashboard.putNumber("WTEST angle error corr", angle_error);
            SmartDashboard.putNumber("TEST angle pwr Raw", angleOutput);
            SmartDashboard.putNumber("Wtarget", _targetAngle);
            SmartDashboard.putNumber("Wrist error", angle_error);
        }
      
        return -angleOutput;
      }
}