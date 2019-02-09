package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;

public class DriveAlign extends Command{

    private double distance = 0;
    private double turn = 0;
    private double drive = 0;
    private Robot _robot;
    NetworkTableEntry tx;
    NetworkTableEntry ta;
    double x;
    double area;
    double angle;
    boolean completed;

    public static DriveDistanceAndDirection goTo;
    public static DriveDistanceAndDirection goTo1;

    public DriveAlign(Robot robot, int target){
        _robot = robot;
        requires(_robot.driveTrain);
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ta = table.getEntry("ta");
        x = tx.getDouble(0.0);
        area = ta.getDouble(0.0);
        angle = _robot.navX.getAngle();
        if(x < 0){
            angle-=180;
        }
        turn = 90 - angle + target - x;
        distance = area*32;
        drive = distance * Math.cos(turn);
    }

    protected void initialize(){
        goTo = new DriveDistanceAndDirection(_robot, drive, turn);
        goTo.start();
        area = ta.getDouble(0.0);
        distance = area;
        //goTo1 = new DriveDistanceAndDirection(_robot, distance, target);
        goTo1.start();
        completed = true;
    }

    protected void execute(){
    }

    protected void end(){
    }

    protected boolean isFinished(){
        return completed;
    }
}