// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Utility.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static frc.robot.Utility.Constants.GlobalConstants.*;

public class RobotContainer 
{



    private final Telemetry logger = new Telemetry(TunerConstants.kMaxSpeed);

    //private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick joystick = new CommandJoystick(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.kMaxSpeed* 0.1)
            .withRotationalDeadband(TunerConstants.kMaxAngularRate * 0.2) // Add deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDesaturateWheelSpeeds(true);

    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {

        
        configureBindings();
        while(!AutoBuilder.isConfigured())
        {

        }
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autoChooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                .withVelocityX(((-1 * joystick.getThrottle() + 3)/4)
                              * (-1 * joystick.getY()) 
                              * TunerConstants.kMaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(((-1 *joystick.getThrottle() + 3)/4)
                              * (-1*joystick.getX()) 
                              * TunerConstants.kMaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(((-1 * joystick.getThrottle() + 3)/4) 
                                    * (joystick.getTwist() > 0 ? -1 : 1)
                                    * Math.pow((-1*joystick.getTwist()),2) 
                                    * TunerConstants.kMaxAngularRate)) // Drive counterclockwise with negative X (left)
            );
        
            

    

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));


        // reset the field-centric heading
        joystick.trigger().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
            .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d(3.16, 4, new Rotation2d(0.0)))))
            );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
