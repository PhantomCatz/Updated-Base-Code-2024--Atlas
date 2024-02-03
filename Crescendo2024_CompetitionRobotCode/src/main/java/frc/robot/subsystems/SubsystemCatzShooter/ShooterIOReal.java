package frc.robot.subsystems.SubsystemCatzShooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.annotation.JsonFormat.Feature;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.LoggedTunableNumber;

public class ShooterIOReal implements ShooterIO {

    //configured from front robot facing perspective
    private final TalonFX SHOOTER_MOTOR_BTM_RT;
    private final TalonFX SHOOTER_MOTOR_TOP_RT;

    private CANSparkMax FEEDER_MOTOR;
    private CANSparkMax FEEDER_MOTOR2;

    //tunable motor velocities
    LoggedTunableNumber shooterVelBtmRt = new LoggedTunableNumber("BtmRtShooter", 98);
    LoggedTunableNumber shooterVelTopRt = new LoggedTunableNumber("TopRtShooter", 98);
    LoggedTunableNumber feederMotor = new LoggedTunableNumber("FeederMotor", -1.0);
    LoggedTunableNumber feederMotor2 = new LoggedTunableNumber("FeederMotor2", -1.0);


    TalonFX[] shooterArray = new TalonFX[4];

    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

                //create new config objects
    private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    private Slot0Configs shooterMtrConfigs = new Slot0Configs();
    SimpleMotorFeedforward shooterFeedForward;
    private final double FeedForwardkV = 3;
    private final double FeedForwardkS = 0;
    public ShooterIOReal() {
                //Drive Motor setup
        
        SHOOTER_MOTOR_BTM_RT = new TalonFX(0);
        SHOOTER_MOTOR_TOP_RT = new TalonFX(7);

        FEEDER_MOTOR = new CANSparkMax(21, MotorType.kBrushless);
        FEEDER_MOTOR.restoreFactoryDefaults();
        FEEDER_MOTOR.setSmartCurrentLimit(DriveConstants.STEER_CURRENT_LIMIT_AMPS);
        FEEDER_MOTOR.setIdleMode(IdleMode.kBrake);
        FEEDER_MOTOR.enableVoltageCompensation(12.0);

        FEEDER_MOTOR2 = new CANSparkMax(70, MotorType.kBrushless); // Changed to 70 original is 1
        FEEDER_MOTOR2.restoreFactoryDefaults();
        FEEDER_MOTOR2.setSmartCurrentLimit(DriveConstants.STEER_CURRENT_LIMIT_AMPS);
        FEEDER_MOTOR2.setIdleMode(IdleMode.kBrake);
        FEEDER_MOTOR2.enableVoltageCompensation(12.0);

                //create shooter mtr array for easier calls
        shooterArray[0] = SHOOTER_MOTOR_BTM_RT;
        shooterArray[1] = SHOOTER_MOTOR_TOP_RT;

            //reset to factory defaults
        SHOOTER_MOTOR_BTM_RT.getConfigurator().apply(new TalonFXConfiguration());
        SHOOTER_MOTOR_TOP_RT.getConfigurator().apply(new TalonFXConfiguration());

    
        talonConfigs.Slot0 = shooterMtrConfigs;
            //current limit
        talonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = DriveConstants.ENABLE_CURRENT_LIMIT;
        talonConfigs.CurrentLimits.SupplyCurrentLimit       = DriveConstants.CURRENT_LIMIT_AMPS;
        talonConfigs.CurrentLimits.SupplyCurrentThreshold   = DriveConstants.CURRENT_LIMIT_TRIGGER_AMPS;
        talonConfigs.CurrentLimits.SupplyTimeThreshold      = DriveConstants.CURRENT_LIMIT_TIMEOUT_SECONDS;
            //neutral mode
        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            //pid
        shooterMtrConfigs.kP = 0.2;
        shooterMtrConfigs.kI = 0.0;
        shooterMtrConfigs.kD = 0.0;
        shooterFeedForward = new SimpleMotorFeedforward(FeedForwardkS, FeedForwardkV);

        
        
        //check if drive motor is initialized correctly
        for(int i=0;i<2;i++){
            initializationStatus = shooterArray[i].getConfigurator().apply(talonConfigs);
             if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID for shooter "+ shooterArray.toString());

        }

        SHOOTER_MOTOR_BTM_RT.setControl(new Follower(SHOOTER_MOTOR_TOP_RT.getDeviceID(), true));

        
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        //System.out.println("Cmd updateInputs");
        inputs.dummyVariable = 1;
        inputs.velocityBtmRT = SHOOTER_MOTOR_BTM_RT.getVelocity().getValue();
        inputs.velocityTopRT = SHOOTER_MOTOR_TOP_RT.getVelocity().getValue();
        Logger.recordOutput("FeedFwrdConstkV", FeedForwardkV);
        Logger.recordOutput("FeedFwrdConstkS", FeedForwardkS);
        inputs.shooterTopPercentOutput = SHOOTER_MOTOR_TOP_RT.getDutyCycle().getValue();
        inputs.shooterBtmPercentOutput = SHOOTER_MOTOR_BTM_RT.getDutyCycle().getValue();
        inputs.shooterTopSupplyCurrent = SHOOTER_MOTOR_TOP_RT.getSupplyCurrent().getValue();
        inputs.shooterBtmSupplyCurrent = SHOOTER_MOTOR_BTM_RT.getSupplyCurrent().getValue();
        inputs.shooterTopStatorCurrent = SHOOTER_MOTOR_TOP_RT.getStatorCurrent().getValue();
        inputs.shooterBtmStatorCurrent = SHOOTER_MOTOR_BTM_RT.getStatorCurrent().getValue();
        inputs.shooterTopTorqueCurrent = SHOOTER_MOTOR_TOP_RT.getTorqueCurrent().getValue();
        inputs.shooterBtmTorqueCurrent = SHOOTER_MOTOR_BTM_RT.getTorqueCurrent().getValue();

        inputs.feederMotorPercentOutput = FEEDER_MOTOR.get();
        inputs.feederMotorVelocity = (FEEDER_MOTOR.getEncoder().getVelocity()/60); //to rps
        inputs.feederMotor2PercentOutput = FEEDER_MOTOR2.get();
        inputs.feederMotor2Velocity = (FEEDER_MOTOR2.getEncoder().getVelocity()/60); //to rps
    }

    @Override
    public void shootWithVelocity() {
        double bottomVelocity = shooterVelBtmRt.get();
        double topVelocity = shooterVelTopRt.get();
        SHOOTER_MOTOR_TOP_RT.setControl(new VelocityVoltage(-(topVelocity + shooterFeedForward.calculate(topVelocity))));
        SHOOTER_MOTOR_BTM_RT.setControl(new VelocityVoltage(bottomVelocity + shooterFeedForward.calculate(bottomVelocity)));
        FEEDER_MOTOR2.set(feederMotor2.get());

    }

    @Override
    public void setShooterDisabled() {
        // System.out.println("CMd off");
        //SHOOTER_MOTOR_BTM_LT.setControl(new DutyCycleOut(0));
        //SHOOTER_MOTOR_BTM_RT.setControl(new DutyCycleOut(0));
        SHOOTER_MOTOR_TOP_RT.setControl(new DutyCycleOut(0));
        SHOOTER_MOTOR_BTM_RT.setControl(new DutyCycleOut(0));
        FEEDER_MOTOR2.set(0);
    }

    public void shootFeederWithVelocity() {
        // FEEDER_MOTOR.set(FEEDER_MOTOR.getEncoder().getVelocity()); *TBD doesn't work 
        // FEEDER_MOTOR.set(feederMotor.getVelocity().speed());
        FEEDER_MOTOR.set(feederMotor.get());
    }
    
    @Override
    public void shootFeederReverse() {
        //FEEDER_MOTOR.set(-FEEDER_MOTOR.getEncoder().getVelocity());
        FEEDER_MOTOR.set(-feederMotor.get());
    }

    @Override
    public void setFeederDisabled() {
        FEEDER_MOTOR.set(0);
    }
}
