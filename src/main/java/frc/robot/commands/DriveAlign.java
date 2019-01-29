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
    boolean completed;

    public static DriveDistanceAndDirection goTo;

    public DriveAlign(Robot robot){
        requires(_robot.driveTrain);
        _robot = robot;
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ta = table.getEntry("ta");
        x = tx.getDouble(0.0);
        turn = 90 - Math.atan(1/Math.tan(x));
        double area = ta.getDouble(0.0);
        distance = area*5;
        drive = distance / Math.cos(x);
    }

    protected void initialize(){
        goTo = new DriveDistanceAndDirection(_robot, drive, turn);
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