package frc.robot.Utility;


import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;

import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants 
{
    public static final double kraken_free_speed = 5800 / 60;
    public static final double neo_free_speed = 5676 / 60;

    //Most drive related constants are in TunerConstants.java, which is part of the CTRE swerve generator
    public static final class GlobalConstants
    {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorcontrollerPort = 1;

        


    }

    public static final class DriveConstants
    {
        //most of these constants are in TunerConstants, but some is used here

        public static final double PathKP = 5;
        public static final double PathKI = 0;
        public static final double PathKD = 0;
        public static final double PathKP_Theta = 5;
        public static final Constraints AlignConstraints = new Constraints(2, 4);
        public static final Constraints AlignConstraints_rot = new Constraints(6, 10);

    }

    public static final class ElevatorConstants
    {
        public static final int elevator_ID = 20;
        public static final int tomahawk_ID = 22;
        public static final int pivot_ID = 26;
        public static final int coral_intake_ID = 27;

        public static final double elevator_ratio = 12;
        public static final double tomahawk_ratio = 20;
        public static final double pivot_ratio = 12;
        public static final double elevator_sprocket_diameter = Units.inchesToMeters(1);
        public static final double pivot_sprocket_diameter = Units.inchesToMeters(0.5);
        

        public static final double elevator_KG = 0.35;
        public static final double elevator_KS = 0;
        public static final double elevator_KV = 0 / (kraken_free_speed * elevator_sprocket_diameter / elevator_ratio);
        public static final double elevator_KP = 0;
        public static final double elevator_KD = 0;

        public static final double tomahawk_KG = 0.55;
        public static final double tomahawk_KS = 0.05;
        public static final double tomahawk_KV = 12/ (kraken_free_speed / tomahawk_ratio);
        public static final double tomahawk_KP = 20;
        public static final double tomahawk_KD = 1;

        public static final double tomahawk_KV_down = 6 / (kraken_free_speed / tomahawk_ratio);
        public static final double tomahawk_KP_down = 3;
        public static final double tomahawk_KD_down = 0;

        public static final double pivot_KG = 0;
        public static final double pivot_KS = 0;
        public static final double pivot_KV = 0 / (kraken_free_speed * pivot_sprocket_diameter / pivot_ratio);
        public static final double pivot_KP = 0;
        public static final double pivot_KD = 0;


        //elevator hard max 5.45
        public static final double elevator_SP_NEUTRAL = 0;
        public static final double elevator_SP_LOAD = 0;
        public static final double elevator_SP_LEVEL1 = 0;
        public static final double elevator_SP_LEVEL2 = 0;
        public static final double elevator_SP_LEVEL3 = 0;
        public static final double elevator_SP_LEVEL4 = 0;

    }

    public static final class LocalizationConstants
    {
        public static String limelight_1_name = "LL_one";
        public static String limelight_2_name = "LL_two";

        public static final double field_length_meters = 17.548;
        public static final double field_width_meters = 8.052;
    
        public static final Translation2d field_center = new Translation2d(field_length_meters / 2, field_width_meters / 2);
        
        public static Pose2d reef_1R = new Pose2d(3.22,3.86,new Rotation2d(0)); 
        public static Pose2d reef_1L = new Pose2d(3.22,4.19,new Rotation2d(0)); 

        public static Pose2d reef_2R = new Pose2d(4.00,2.84,new Rotation2d(60)); 
        public static Pose2d reef_2L = new Pose2d(3.71,3.01,new Rotation2d(60)); 

        public static Pose2d reef_3R = new Pose2d(5.26,3.02,new Rotation2d(120)); 
        public static Pose2d reef_3L = new Pose2d(4.97,2.85,new Rotation2d(120)); 

        public static Pose2d reef_4R = new Pose2d(5.75,4.19,new Rotation2d(180)); 
        public static Pose2d reef_4L = new Pose2d(5.75,3.86,new Rotation2d(180)); 

        public static Pose2d reef_5R = new Pose2d(4.98,5.20,new Rotation2d(240)); 
        public static Pose2d reef_5L = new Pose2d(5.26,5.03,new Rotation2d(240)); 

        public static Pose2d reef_6R = new Pose2d(3.72,5.03,new Rotation2d(300)); 
        public static Pose2d reef_6L = new Pose2d(4.01,5.19,new Rotation2d(300)); 

        
        //all poses are for blue alliance. If you create a pose, add it to this function to transform it for red.
        public static void alliance_transform()
        {
            reef_1R = reef_1R.rotateAround(field_center, Rotation2d.k180deg);
            reef_1L = reef_1L.rotateAround(field_center, Rotation2d.k180deg);

            reef_2R = reef_2R.rotateAround(field_center, Rotation2d.k180deg);
            reef_2L = reef_2L.rotateAround(field_center, Rotation2d.k180deg);

            reef_3R = reef_3R.rotateAround(field_center, Rotation2d.k180deg);
            reef_3L = reef_3L.rotateAround(field_center, Rotation2d.k180deg);

            reef_4R = reef_4R.rotateAround(field_center, Rotation2d.k180deg);
            reef_4L = reef_4L.rotateAround(field_center, Rotation2d.k180deg);

            reef_5R = reef_5R.rotateAround(field_center, Rotation2d.k180deg);
            reef_5L = reef_5L.rotateAround(field_center, Rotation2d.k180deg);

            reef_6R = reef_6R.rotateAround(field_center, Rotation2d.k180deg);
            reef_6L = reef_6L.rotateAround(field_center, Rotation2d.k180deg);

        }

    }
}
