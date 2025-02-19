package frc.robot.Utility;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Utility.Constants.LocalizationConstants;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;



public class LimelightPose
{
    static String LL1 = LocalizationConstants.limelight_1_name;
    static String LL2 = LocalizationConstants.limelight_2_name;
    static VisionMeasurement measurement = new VisionMeasurement();
    public LimelightPose()
    {
    
    }

    public static void evaluate(Pose2d pose, ChassisSpeeds speeds, Consumer<VisionMeasurement> applyPose)
    {
        /*
        An Optional<T> object holds a value of type T that may or may not exist, while avoiding null pointer exceptions
        In this case the evaluate limelight returns an Optional<Pose2d> because it may not see any targets

        One method of an optional is Optional<T>.ifPresent(Consumer<T>)
        If the optional is present, the Consumer function is run and is passed the value of the optional
        Since we know the type T, whatever parameter I use is assumed to be that type. So I can make a lambda with (p) -> and it is known to be the Pose2d
        */
        evaluate_single_LL_rev2(LL1, pose, speeds).ifPresent(applyPose);
        evaluate_single_LL_rev2(LL2, pose, speeds).ifPresent(applyPose);

    }

    private static Optional<VisionMeasurement> evaluate_single_LL_rev2(String LL, Pose2d pose, ChassisSpeeds speeds)
    {
        LimelightHelpers.SetRobotOrientation(LL, pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        //MegaTag 1 is usually less reliable, but it can be allowed to second-guess the gyro if it is high quality
        //MegaTag 2 is more reliable in a variety of cases, but needs to know the gyro, therefore it can't be trusted to update the bot's angle
        LimelightHelpers.PoseEstimate MT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LL);
        LimelightHelpers.PoseEstimate MT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL);

        double PoseDiff_MT1 = MT1.pose.getTranslation().getDistance(pose.getTranslation());
        double PoseDiff_MT2 = MT2.pose.getTranslation().getDistance(pose.getTranslation());

        double TagCount_MT1 = MT1.tagCount;
        double TagCount_MT2 = MT2.tagCount;

        double AvgArea_MT1 = MT1.avgTagArea;
        double AvgArea_MT2 = MT2.avgTagArea;
        
        double velocityTranslation = Math.sqrt(Math.pow(speeds.vxMetersPerSecond,2) + Math.pow(speeds.vyMetersPerSecond,2));
        double velocityRotation = Math.toDegrees(speeds.omegaRadiansPerSecond);

        double xyDev;
        double rotDev;
        
        //I'm only trusting MT1 when it's really good, but if it is I'll trust it pretty solidly and let it update the heading
        if(TagCount_MT1 >= 2 && AvgArea_MT1 > 0.2 && PoseDiff_MT1 < 0.5)
        {
            if(velocityTranslation < 1 )
            {
                xyDev = 0.2;
            }
            else
            {
                xyDev = 0.5;
            }

            if(velocityRotation < 15)
            {
                rotDev = 5;
            }
            else if(velocityRotation < 45)
            {
                rotDev = 10;
            }
            else
            {
                rotDev = 20;
            }

            measurement.set(MT1.pose, MT1.timestampSeconds, xyDev, rotDev, LL);
            return Optional.of(measurement);
        }

        if(velocityRotation < 90)
        {
            if(TagCount_MT2 >= 2 && AvgArea_MT2 > 0.1 )
            {
                xyDev = 0.2;
                measurement.set(MT2.pose, MT2.timestampSeconds, xyDev,9999999, LL);
                return Optional.of(measurement);
            }

            if(TagCount_MT2 == 1 && AvgArea_MT2 > 0.6 && PoseDiff_MT2 < 0.5)
            {
                xyDev = 0.5;
                measurement.set(MT2.pose, MT2.timestampSeconds,xyDev,9999999, LL);
                return Optional.of(measurement);
            }

        }

        return Optional.empty();
    }

}
