package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.controls.Follower;
import static frc.robot.Utility.Constants.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase
{
    TalonFX elevator_motor = new TalonFX(20);
    TalonFX elevator_motor_2 = new TalonFX(21);
    

    TalonFXConfiguration elevator_config = new TalonFXConfiguration();

    Slot0Configs elevatorSlot0 = elevator_config.Slot0;
    CurrentLimitsConfigs currentLimit = elevator_config.CurrentLimits;
    MotionMagicConfigs motionMagicConfigs =  elevator_config.MotionMagic;
    
    MotionMagicExpoVoltage elevator_motion_request = new MotionMagicExpoVoltage(0);

    
    



    double elevator_position;
    double elevator_setpoint;
    double wrist_angle;

    public Elevator()
    {
        
        elevator_motor_2.setControl(new Follower(elevator_motor.getDeviceID(), false));
        currentLimit.StatorCurrentLimit = 10; //we'll definitely raise this, but should help not break anything at first
        
        elevator_config.Feedback.SensorToMechanismRatio = 1; //convert rotations to inches

        elevatorSlot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorSlot0.kG = 0; // Add 0 V output to overcome gravity
        elevatorSlot0.kS = 0; // Add 0 V output to overcome static friction
        elevatorSlot0.kV = 0; // A velocity target of 1 in/s results in 0 V output
        elevatorSlot0.kA = 0; // An acceleration of 1 in/s/s requires 0 V output
        elevatorSlot0.kP = 0; // A position error of 0 in results in 12 V output
        elevatorSlot0.kI = 0; // no output for integrated error
        elevatorSlot0.kD = 0; // A velocity error of 1 in/s results in 0 V output

        // set Motion Magic Expo settings
        
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0; // kV is around 0 V/(in/s)
        motionMagicConfigs.MotionMagicExpo_kA = 0; // Use a kA of 0 V/(in/s/s)
        

        elevator_motor.getConfigurator().apply(elevator_config);

        
    }    

    @Override
    public void periodic() 
    {
        elevator_position = elevator_motor.getPosition(true).getValueAsDouble();
    }


    void goToPosition_wrist(double angle)
    {
        //dont let the wrist crash into the frame at low angles
        if(elevator_position < elevator_danger || elevator_setpoint < elevator_danger)
        {
            angle = wrist_safe_up;
        }
    }

    void goToPosition_elevator(double position)
    {
        elevator_setpoint = position;
        elevator_motor.setControl(elevator_motion_request.withPosition(position).withEnableFOC(true));
    }
}
