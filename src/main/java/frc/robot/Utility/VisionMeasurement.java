package frc.robot.Utility;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

public class VisionMeasurement 
{
    public Pose2d pose;
    public double timestamp;
    public double timestampFPGA;
    public double xyStdDev;
    public double rotStdDev;
    public Vector<N3> StdDevs;
    public String LL_name;

    public VisionMeasurement(Pose2d pose, double timestamp, double xyStdDev, double rotStdDevs, String LL_name)
    {
        this.pose = pose;
        this.timestamp = timestamp;
        timestampFPGA = Utils.fpgaToCurrentTime(timestamp);

        VecBuilder.fill(xyStdDev, rotStdDev);

        this.LL_name = LL_name;
    }

    public VisionMeasurement(){}

    public void set(Pose2d pose, double timestamp, double xyStdDev, double rotStdDev, String LL_name)
    {
        this.pose = pose;
        this.timestamp = timestamp;
        timestampFPGA = Utils.fpgaToCurrentTime(timestamp);

        VecBuilder.fill(xyStdDev, rotStdDev);

        this.LL_name = LL_name;
    }
}
