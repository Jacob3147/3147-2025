// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Utility.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

import static frc.robot.Utility.Constants.GlobalConstants.*;

public class RobotContainer 
{



    private final Telemetry logger = new Telemetry(TunerConstants.kMaxSpeed);

    //private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandGenericHID buttonBox = new CommandGenericHID(1);
    private final CommandGenericHID buttonBox2 = new CommandGenericHID(2);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.kMaxSpeed* 0.1)
            .withRotationalDeadband(TunerConstants.kMaxAngularRate * 0.2) // Add deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true);

    private final Elevator elevator = new Elevator(() -> joystick.getThrottle());

    private final SendableChooser<Command> autoChooser;

    


    public RobotContainer() {

        NamedCommands.registerCommand("Go to L1", Commands.runOnce(() -> elevator.state = ElevatorState.L1_CORAL));
        NamedCommands.registerCommand("Go to L2", Commands.runOnce(() -> elevator.state = ElevatorState.L2_CORAL));
        NamedCommands.registerCommand("Go to L3", Commands.runOnce(() -> elevator.state = ElevatorState.L3_CORAL));
        NamedCommands.registerCommand("Score Coral", Commands.runOnce(() -> elevator.reverse_coral_intake()));
        NamedCommands.registerCommand("Neutral", Commands.runOnce(() -> elevator.state = ElevatorState.NEUTRAL));
        
        configureBindings();
        while(!AutoBuilder.isConfigured())
        {
            //wait
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


        
        


        //Button box decides which state will be next
        buttonBox.button(1).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.L4_CORAL));
        buttonBox.button(2).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.L3_CORAL));
        buttonBox.button(3).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.L2_CORAL));
        buttonBox.button(4).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.L1_CORAL));
        buttonBox.button(5).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.NET));
        buttonBox.button(6).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.LOADING));
        buttonBox.button(7).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.PROCESSOR));
        buttonBox.button(8).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.GROUND_ALGAE));
        buttonBox.button(9).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.HIGH_REEF_ALGAE));
        buttonBox.button(10).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.LOW_REEF_ALGAE));
        
        //Joystick thumb button goes to the state
        joystick.button(2)
                .and(() -> elevator.state == ElevatorState.NEUTRAL).onTrue(
                    Commands.runOnce(() -> elevator.execQueued()));
        
        joystick.button(11).or(joystick.button(12)).onTrue(Commands.runOnce(() -> elevator.state = ElevatorState.NEUTRAL));

        //If the state is an intake, spin the correct intake while held, then stop when released and go to neutral
        joystick.button(2)
                .and(() -> elevator.state == ElevatorState.LOADING).whileTrue(
                    Commands.startEnd(
                        () -> elevator.start_coral_intake(),
                        () -> elevator.state = ElevatorState.NEUTRAL));

        joystick.button(2)
                    .and(() -> elevator.state == ElevatorState.GROUND_ALGAE).whileTrue(
                        Commands.startEnd(
                        () -> elevator.start_algae_intake(),
                        () -> elevator.state = ElevatorState.NEUTRAL));

        joystick.button(2)
                .and(() -> elevator.state == ElevatorState.HIGH_REEF_ALGAE).whileTrue(
                    Commands.startEnd(
                        () -> elevator.start_algae_intake(),
                        () -> elevator.state = ElevatorState.NEUTRAL));

        joystick.button(2)
                .and(() -> elevator.state == ElevatorState.LOW_REEF_ALGAE).whileTrue(
                    Commands.startEnd(
                        () -> elevator.start_algae_intake(),
                        () -> elevator.state = ElevatorState.NEUTRAL));


        joystick.trigger()
                .and(() -> elevator.state == ElevatorState.L1_CORAL).onTrue(
                    Commands.runOnce(() -> elevator.soft_coral_eject())
                );

        joystick.trigger()
                .and(() -> elevator.state == ElevatorState.L2_CORAL).onTrue(
                    Commands.runOnce(() -> elevator.reverse_coral_intake())
                );

        joystick.trigger()
                .and(() -> elevator.state == ElevatorState.L3_CORAL).onTrue(
                    Commands.runOnce(() -> elevator.reverse_coral_intake())
                );


        joystick.trigger()
                .and(() -> elevator.state == ElevatorState.PROCESSOR).onTrue(
                    Commands.runOnce(() -> elevator.reverse_coral_intake())
                );

        joystick.button(7).or(joystick.button(8)).onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void elevator_neutral()
    {
        elevator.state = ElevatorState.NEUTRAL;
    }
}
