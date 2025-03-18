package frc.robot.subsystems;


import static frc.robot.Utility.Constants.ElevatorConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase
{
    public enum ElevatorState
    {
        START_1,
        START_2,
        START_3,
        NEUTRAL,
        LOADING,
        L1_CORAL,
        L2_CORAL,
        L3_CORAL,
        L4_CORAL,
        LOW_ALGAE,
        HIGH_ALGAE
    }

    public ElevatorState state = ElevatorState.START_1;
    public ElevatorState queued = ElevatorState.NEUTRAL;


    TalonFX elevator;
    TalonFXConfiguration elevator_config;
    MotionMagicExpoVoltage elevator_motion_request = new MotionMagicExpoVoltage(0);
    double elevator_position;
    double elevator_setpoint;


    TalonFX tomahawk;
    TalonFXConfiguration tomahawk_config;
    MotionMagicExpoVoltage tomahawk_motion_request = new MotionMagicExpoVoltage(0);
    double tomahawk_position;
    double tomahawk_setpoint;
    DutyCycleEncoder tomahawk_encoder;

    double tomahawk_encoder_position;
    double tomahawk_kraken_position;
    double tomahawk_offset = -0.7257;


    TalonFXS coral_intake;
    TalonFXSConfiguration coral_intake_config;
    VoltageOut coral_volt = new VoltageOut(0);
    

    TalonFX pivot;
    TalonFXConfiguration pivot_config;
    double pivot_position;
    double pivot_setpoint;
    DutyCycleEncoder pivot_encoder;

    double pivot_kraken_position;
    double pivot_encoder_position;
    double pivot_offset = -.26;


    Supplier<Double> throttleSupplier;
    double throttle;

    VoltageOut voltage = new VoltageOut(0);

    DigitalInput beam_break;
    BooleanSupplier beam_break_supplier;

    
    MotionMagicConfigs elevatorMotionMagicConfigs;
    public Elevator(Supplier<Double> throttleSupplier)
    {

        this.throttleSupplier = throttleSupplier;
        tomahawk_encoder = new DutyCycleEncoder(0);
        pivot_encoder = new DutyCycleEncoder(1);
        beam_break = new DigitalInput(2);


        tomahawk       = new TalonFX(tomahawk_ID,        "rio");
        elevator       = new TalonFX (elevator_ID,       "rio");
        coral_intake   = new TalonFXS(coral_intake_ID,   "rio");
        pivot          = new TalonFX(pivot_ID,           "rio");


        tomahawk_config       = new TalonFXConfiguration();
        Slot0Configs tomahawkSlot0 = tomahawk_config.Slot0;
        Slot1Configs tomahawkSlot1 = tomahawk_config.Slot1;
        CurrentLimitsConfigs tomahawkCurrentLimit = tomahawk_config.CurrentLimits;
        MotionMagicConfigs tomahawkMotionMagicConfigs =  tomahawk_config.MotionMagic;
        FeedbackConfigs tomahawkFeedback = tomahawk_config.Feedback;
        MotorOutputConfigs tomahawkOutputConfig = tomahawk_config.MotorOutput;
        

        elevator_config = new TalonFXConfiguration();
        Slot0Configs elevatorSlot0 = elevator_config.Slot0;
        Slot1Configs elevatorSlot1 = elevator_config.Slot1;
        CurrentLimitsConfigs elevatorCurrentLimit = elevator_config.CurrentLimits;
        elevatorMotionMagicConfigs =  elevator_config.MotionMagic;
        FeedbackConfigs elevatorFeedback = elevator_config.Feedback;
        MotorOutputConfigs elevatorOutputConfig = elevator_config.MotorOutput;

        


        coral_intake_config  = new TalonFXSConfiguration();
        CurrentLimitsConfigs coral_intake_CurrentLimit = coral_intake_config.CurrentLimits;
        MotorOutputConfigs coral_intake_outputConfig = coral_intake_config.MotorOutput;
        coral_intake_config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        pivot_config = new TalonFXConfiguration();
        Slot0Configs pivotSlot0 = pivot_config.Slot0;
        CurrentLimitsConfigs pivotCurrentLimit = pivot_config.CurrentLimits;
        MotionMagicConfigs pivotMotionMagicConfigs =  pivot_config.MotionMagic;
        FeedbackConfigs pivotFeedback = pivot_config.Feedback;
        MotorOutputConfigs pivotOutputConfig = pivot_config.MotorOutput;


        
        tomahawkSlot0.withGravityType(GravityTypeValue.Arm_Cosine)
                     .withKV(tomahawk_KV).withKS(tomahawk_KS).withKG(tomahawk_KG)
                     .withKP(tomahawk_KP).withKD(tomahawk_KD);
        tomahawkSlot1.withGravityType(GravityTypeValue.Arm_Cosine)
                     .withKV(tomahawk_KV_down).withKS(tomahawk_KS).withKG(tomahawk_KG)
                     .withKP(tomahawk_KP_down).withKD(tomahawk_KD_down);

        tomahawkCurrentLimit.withStatorCurrentLimit(60);
        tomahawkMotionMagicConfigs.withMotionMagicAcceleration(1)
                          .withMotionMagicCruiseVelocity(1);
        tomahawkFeedback.SensorToMechanismRatio = tomahawk_ratio;
        tomahawkOutputConfig.NeutralMode = NeutralModeValue.Brake;
        tomahawkOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;


        elevatorSlot0.withGravityType(GravityTypeValue.Elevator_Static)
                     .withKV(elevator_KV).withKS(elevator_KS).withKG(elevator_KG)
                     .withKP(elevator_KP).withKD(elevator_KD);
        elevatorCurrentLimit.withStatorCurrentLimit(60);
        elevatorMotionMagicConfigs.withMotionMagicAcceleration(3)
                          .withMotionMagicCruiseVelocity(3);
        
        elevatorFeedback.SensorToMechanismRatio = elevator_ratio;
        elevatorOutputConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;


        coral_intake_CurrentLimit.withStatorCurrentLimit(60);
        coral_intake_outputConfig.Inverted = InvertedValue.Clockwise_Positive;

        
        pivotSlot0.withGravityType(GravityTypeValue.Elevator_Static)
                     .withKV(pivot_KV).withKS(pivot_KS).withKG(pivot_KG)
                     .withKP(pivot_KP).withKD(pivot_KD);
        pivotCurrentLimit.withStatorCurrentLimit(40);
        pivotMotionMagicConfigs.withMotionMagicAcceleration(1)
                          .withMotionMagicCruiseVelocity(1);
        pivotFeedback.SensorToMechanismRatio = pivot_ratio;
        pivotOutputConfig.NeutralMode = NeutralModeValue.Brake;
        pivotOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

        
        tomahawk    .getConfigurator().apply(tomahawk_config);
        elevator    .getConfigurator().apply(elevator_config);
        coral_intake.getConfigurator().apply(coral_intake_config);
        pivot       .getConfigurator().apply(pivot_config);


        tomahawk.setPosition(tomahawk_encoder.get() + tomahawk_offset);
        pivot.setPosition(pivot_encoder.get() + pivot_offset);

    }    

    double throttle_adjust_coral_wrist;
    @Override
    public void periodic() 
    {
        /*
        if(state = ElevatorState.LOADING && beam_break.get())
        {
            state = ElevatorState.NEUTRAL;
        }
         */

        if(tomahawk.getPosition(true).getValueAsDouble() > tomahawk_setpoint)
        {
            tomahawk_motion_request.withSlot(1);
        }
        else
        {
            tomahawk_motion_request.withSlot(0);
        }

        throttle = throttleSupplier.get();
        voltage.Output = throttle*5;
        pivot.setControl(voltage);
        
        //throttle_adjust_coral_wrist = throttle * -1 * Units.degreesToRadians(5);

        

        coral_intake.setControl(coral_volt);
        elevatorControl(elevator_setpoint);
        tomahawkControl(tomahawk_setpoint);


        //Kraken encoder
        elevator_position = elevator.getPosition(true).getValueAsDouble();
        tomahawk_kraken_position = tomahawk.getPosition(true).getValueAsDouble();
        pivot_kraken_position = pivot.getPosition(true).getValueAsDouble();

        //thru bore
        tomahawk_position = Units.rotationsToRadians(tomahawk_encoder.get()) + tomahawk_offset;

        pivot_encoder_position = /*-1 * */pivot_encoder.get();
        
        
        SmartDashboard.putNumber("elevator", elevator_position);
        SmartDashboard.putNumber("tomahawk kraken", tomahawk_kraken_position);
        SmartDashboard.putNumber("tomahawk encoder", tomahawk_position);
        SmartDashboard.putNumber("pivot kraken", pivot_kraken_position);
        SmartDashboard.putNumber("pivot encoder", pivot_encoder_position);
        SmartDashboard.putNumber("throttle", throttle);

        SmartDashboard.putBoolean("beam break", beam_break.get());

        SmartDashboard.putString("state", state.toString());
        SmartDashboard.putString("queued", queued.toString());

        switch(state)
        {
            case START_1:
                elevator_setpoint = 0.2;
                tomahawk_setpoint = 0;
                unstow();
                break;

            case START_2:
                elevator_setpoint = 0.2;
                tomahawk_setpoint = 0;
                unstow();
                break;

            case START_3:
                elevator_setpoint = 0.2;
                tomahawk_setpoint = 0;
                unstow();
                break;

            case NEUTRAL:
                elevator_setpoint = 0.2;
                tomahawk_setpoint = -0.15;
                coral_volt.Output = 0;
                break;

            case LOADING:
                elevator_setpoint = 0.2;
                tomahawk_setpoint = -0.23;
                coral_volt.Output = 3;
                break;

            case L1_CORAL:
                elevator_setpoint = 1;
                tomahawk_setpoint = -0.18;
                coral_volt.Output = 0;
                break;

            case L2_CORAL:
                elevator_setpoint = 2.5;
                tomahawk_setpoint = -0.2;
                coral_volt.Output = 0;
                break;

            case L3_CORAL:
                elevator_setpoint = 4.5;
                tomahawk_setpoint = -0.2;
                coral_volt.Output = 0;
                break;

            case L4_CORAL:
                elevator_setpoint = 5.4;
                tomahawk_setpoint = 0.05;
                coral_volt.Output = 0;
                break;

            case HIGH_ALGAE:
                elevator_setpoint = 0.5;
                tomahawk_setpoint = 0;
                coral_volt.Output = 12;
                break;

            case LOW_ALGAE:
                elevator_setpoint = 0;
                tomahawk_setpoint = -0.1;
                coral_volt.Output = 12;
                break;

            default:
                break;
            
        }
       
    }

 
    public void execQueued()
    {
        state = queued;
    }

    public void score()
    {
        switch(state) {
            case L1_CORAL:
                coral_volt.Output = 3;
                break;

            case L2_CORAL:
                coral_volt.Output = 6;
                break;
                

            case L3_CORAL:
                coral_volt.Output = 6;
                break;

            case L4_CORAL:
                coral_volt.Output = -6;
                break;

            case NEUTRAL:
                break;

            case LOADING:
                break;

            case START_1:
            case START_2:
            case START_3:
                unstow();
                break;
            default:
                break;
        }
    }

    

    void unstow()
    {
        state = ElevatorState.NEUTRAL;
    }

    double algae_PID_out;
    double algae_FF_out;
    

    void elevatorControl(double SP)
    {
        elevator_motion_request.Position = SP;
        elevator.setControl(elevator_motion_request);
    }

    void tomahawkControl(double SP)
    {
        tomahawk_motion_request.Position = SP;
        tomahawk.setControl(tomahawk_motion_request);
    }

}
