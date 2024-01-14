/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.Utils.Conversions;

public class CatzSwerveModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private PIDController m_PID;
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
                

    private final double kP = 0.15; //cuz error is in tenths place so no need to mutiply kp value
    private final double kI = 0.0;
    private final double kD = 0.0015;

    private double m_wheelOffset;

    private int m_index;

    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double offset, int index) {
        this.m_index = index;

        switch (CatzConstants.currentMode) {
            case REAL: io = 
                    new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel);
                break;

            case SIM : io = 
                    new ModuleIOSim();
                break;
            default : io = 
                    new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel) {};
                break;
        }

        m_PID = new PIDController(kP, kI, kD);

        m_wheelOffset = offset;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module " + Integer.toString(m_index), inputs);

        //Logging outputs
        Logger.recordOutput("absenctorad" + Integer.toString(m_index) , getAbsEncRadians());

        SmartDashboard.putNumber("absenctorad" + Integer.toString(m_index) , getAbsEncRadians());
        SmartDashboard.putNumber("angle" + Integer.toString(m_index) , getCurrentRotation().getDegrees());
    }

    //----------------------------------------Setting pwr methods-------------------------------
    public void setSteerPower(double pwr) {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM) {
           io.setSteerSimPwrIO(pwr);
        }
        else {       
            io.setSteerPwrIO(pwr);
        }
    }

    public void setDriveVelocity(double velocity) {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM) {
            
        }
        else {
            io.setDriveVelocityIO(velocity);
        }
    }

    public void stopDriving() {
        io.setDrivePwrPercentIO(0.0);
    }
    //----------------------------------Util Methods catzswerve------------------------
    public double getDrvDistanceRaw() {
        return inputs.driveMtrSensorPosition;
    }

    public void setCoastMode() {
        io.setSteerCoastModeIO();
    }

    public void setBrakeMode() {
        io.setSteerBrakeModeIO();
    }

    public double getDrvVelocity() {
        return inputs.driveMtrVelocity;
    }
    
    private double getAbsEncRadians() {
        return (inputs.magEncoderValue - m_wheelOffset) * 2 * Math.PI;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        //calculate turn pwr percent
        double steerPIDpwr = - m_PID.calculate(getAbsEncRadians(), state.angle.getRadians()); 
        setSteerPower(steerPIDpwr);

        //calculate drive pwr
        double driveRPS = Conversions.MPSToRPS(state.speedMetersPerSecond);
        //ff drive control
        //double driveFeedforwardFalcon = m_driveFeedforward.calculate(state.speedMetersPerSecond);
        //set drive velocity
        setDriveVelocity(driveRPS);

        if(m_index == 1) {
            //System.out.println("Target " + m_index + ": " + state);
            //System.out.println("Current " + m_index + ": " + getModuleState());
        }
        //logging
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/current roation" , getAbsEncRadians());
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/target Angle", state.angle.getRadians());
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/target velocity", driveRPS);
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/current velocity", getModuleState().speedMetersPerSecond);
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/turn power", steerPIDpwr);
    }

    //optimze wheel angles before sending to setdesiredstate method for logging
    public SwerveModuleState optimizeWheelAngles(SwerveModuleState unoptimizedState) {
        SwerveModuleState optimizedState = CatzMathUtils.optimize(unoptimizedState, getCurrentRotation()); 
        return optimizedState;
    }

    public void resetDriveEncs() {
        io.setDrvSensorPositionIO(0.0);
    }

    //inputs the rotation object as radian conversion
    public Rotation2d getCurrentRotation() {
        return new Rotation2d(getAbsEncRadians());
    }

    public SwerveModuleState getModuleState() {
        double velocity = Conversions.RPSToMPS(inputs.driveMtrVelocity);
        
        return new SwerveModuleState(velocity, getCurrentRotation());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
    }
    
    public double getDriveDistanceMeters() {
        // seconds cancels out
        return Conversions.RPSToMPS(inputs.driveMtrSensorPosition);
    }
}
