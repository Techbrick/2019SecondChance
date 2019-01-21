package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.DistancePid;
import frc.robot.Robot;
import frc.robot.TurnPid;

public class DriveAlign extends Command{

    private double distance = 0;
    private double turn = 0;
    private double drive = 0;
    private Robot _robot;
    NetworkTableEntry tx;
    NetworkTableEntry ta;
    private int stoppedCounter;
    private boolean testCompleted;
    private DistancePid _distancePid;
    private TurnPid _turnPid;
    double x;

    public DriveAlign(Robot robot){
        requires(_robot.driveTrain);
        _robot = robot;
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ta = table.getEntry("ta");
        x = tx.getDouble(0.0);
        turn = 90 - Math.atan(1/Math.tan(x));
        //read values periodically
        double area = ta.getDouble(0.0);
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightArea", area);
        distance = 25.64 / 30 - 0.29 * area / 30;
        drive = distance / Math.cos(x);
    }

    protected void initialize(){
        testCompleted = false;
        stoppedCounter = 0;
        _distancePid = new DistancePid( _robot);
        _distancePid.SetTargetDistance(drive);
        _turnPid = new TurnPid(_robot);
        _turnPid.SetTargetAngle(turn);
    }

    protected void execute(){
        double power = _distancePid.GetDistancePidOutput();
        double turnPower = _turnPid.GetAnglePidOutput(_robot.navX.getYaw());
        _robot.driveTrain.Move(power - turnPower, power + turnPower); 

        if (power + turnPower == 0){
            stoppedCounter ++;
        }else{
            stoppedCounter = 0;
        }
        if (stoppedCounter > 5){
            testCompleted = true;
        }
    }

    protected void end(){
    }

    protected boolean isFinished(){
        return testCompleted;
    }
}