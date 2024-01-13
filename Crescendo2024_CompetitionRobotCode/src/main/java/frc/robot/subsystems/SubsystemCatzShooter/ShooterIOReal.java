package frc.robot.subsystems.SubsystemCatzShooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.LoggedTunableNumber;

public class ShooterIOReal implements ShooterIO {

    //configured from front robot facing perspective
    private final TalonFX SHOOTER_MOTOR_BTM_LT;
    private final TalonFX SHOOTER_MOTOR_BTM_RT;
    private final TalonFX SHOOTER_MOTOR_TOP_RT;
    private final TalonFX SHOOTER_MOTOR_TOP_LT;

    //tunable motor velocities
    LoggedTunableNumber shooterVelBtmLt = new LoggedTunableNumber("BtmLtShooter", 12);
    LoggedTunableNumber shooterVelBtmRt = new LoggedTunableNumber("BtmRtShooter", 12);
    LoggedTunableNumber shooterVelTopRt = new LoggedTunableNumber("TopRtShooter", 12);
    LoggedTunableNumber shooterVelTopLt = new LoggedTunableNumber("TopLtShooter", 12);

    TalonFX[] shooterArray = new TalonFX[4];

    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

                //create new config objects
    private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    private Slot0Configs shooterMtrConfigs = new Slot0Configs();

    public ShooterIOReal() {
                //Drive Motor setup
        SHOOTER_MOTOR_BTM_LT = new TalonFX(6);
        SHOOTER_MOTOR_BTM_RT = new TalonFX(7);
        SHOOTER_MOTOR_TOP_RT = new TalonFX(8);
        SHOOTER_MOTOR_TOP_LT = new TalonFX(9);

                //create shooter mtr array for easier calls
        shooterArray[0] = SHOOTER_MOTOR_BTM_LT;
        shooterArray[1] = SHOOTER_MOTOR_BTM_RT;
        shooterArray[2] = SHOOTER_MOTOR_TOP_RT;
        shooterArray[3] = SHOOTER_MOTOR_TOP_LT;

            //reset to factory defaults
        SHOOTER_MOTOR_BTM_LT.getConfigurator().apply(new TalonFXConfiguration());
        SHOOTER_MOTOR_BTM_RT.getConfigurator().apply(new TalonFXConfiguration());
        SHOOTER_MOTOR_TOP_RT.getConfigurator().apply(new TalonFXConfiguration());
        SHOOTER_MOTOR_TOP_LT.getConfigurator().apply(new TalonFXConfiguration());

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
        shooterMtrConfigs.kP = 0.1;
        shooterMtrConfigs.kI = 0.0;
        shooterMtrConfigs.kD = 0.0;

        //check if drive motor is initialized correctly
        for(int i=0;i<5;i++){
            initializationStatus = shooterArray[i].getConfigurator().apply(talonConfigs);
            if(initializationStatus.isOK())
                break;
            else if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID for shooter" );
        }
        
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.dummyVariable = 1;
        inputs.velocityBtmLT = SHOOTER_MOTOR_BTM_LT.getVelocity().getValue();
        inputs.velocityBtmRT = SHOOTER_MOTOR_BTM_RT.getVelocity().getValue();
        inputs.velocityTopRT = SHOOTER_MOTOR_TOP_RT.getVelocity().getValue();
        inputs.velocityTopLT = SHOOTER_MOTOR_TOP_LT.getVelocity().getValue();
    }

    @Override
    public void shootWithVelocity() {
        SHOOTER_MOTOR_BTM_LT.setControl(new VelocityDutyCycle(shooterVelBtmLt.get()));
        SHOOTER_MOTOR_BTM_RT.setControl(new VelocityDutyCycle(shooterVelBtmRt.get()));
        SHOOTER_MOTOR_TOP_RT.setControl(new VelocityDutyCycle(shooterVelTopRt.get()));
        SHOOTER_MOTOR_TOP_LT.setControl(new VelocityDutyCycle(shooterVelBtmLt.get()));
    }

    @Override
    public void setShooterDisabled() {
        for(int i=0;i<4;i++){
            shooterArray[i].setControl(new VelocityDutyCycle(0));
        }
    }
}
