/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.Utils.Conversions;


public class CatzSwerveModule {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged   inputs = new ModuleIOInputsAutoLogged();

    private ProfiledPIDController m_ProfiledPID;
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
                

    private final double kP = 0.55; //cuz error is in tenths place so no need to mutiply kp value
    private final double kI = 0.0;
    private final double kD = 0.0175;

    private double m_wheelOffset;

    private int m_index;

    private VelocityTorqueCurrentFOC velocitySetter = new VelocityTorqueCurrentFOC(0);

    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double offset, int index) {
        this.m_index = index;

        switch (CatzConstants.currentMode) {
            case REAL: io = 
                    new ModuleIOReal(driveMotorID, 
                                     steerMotorID, 
                                     encoderDIOChannel);
                break;

            case SIM : io = 
                    new ModuleIOSim();
                break;
            default : io = 
                    new ModuleIOReal(driveMotorID, 
                                     steerMotorID, 
                                     encoderDIOChannel) {};
                break;
        }

        m_ProfiledPID = new ProfiledPIDController(kP, kI, kD, 
                                                    new TrapezoidProfile.Constraints(3.0, 3.0)); //tbd need to find out max velocity and acceleration for steering mtrs

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

    //----------------------------------------Setting pwr methods
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
    //TBD put in autobalance file
    /*Auto Balance */
    public void reverseDrive(Boolean reverse) {
        io.reverseDriveIO(reverse);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {

        //optimize wheel angles
        state = CatzMathUtils.optimize(state, getCurrentRotation());

        //calculate turn pwr percent
        double steerPIDpwr = - m_ProfiledPID.calculate(getAbsEncRadians(), state.angle.getRadians()); 
        setSteerPower(steerPIDpwr);

        //calculate drive pwr
        double drivePwrVelocityMPS = Conversions.MPSToFalcon(state.speedMetersPerSecond, 
                                                             DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, 
                                                             DriveConstants.SDS_L2_GEAR_RATIO); //to set is as a gear reduction not an overdrive
        //ff drive control
        double driveFeedforwardMPS = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        setDriveVelocity(drivePwrVelocityMPS + driveFeedforwardMPS);
        

        if(m_index == 1) {
            System.out.println("Target " + m_index + ": " + state);
            System.out.println("Current " + m_index + ": " + getModuleState());
        }
        //logging
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/current roation" , getAbsEncRadians());
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/target Angle", state.angle.getRadians());
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/target velocity", drivePwrVelocityMPS);
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/current velocity", getModuleState().speedMetersPerSecond);
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/turn power", steerPIDpwr);
    }

    public void resetDriveEncs() {
        io.setDrvSensorPositionIO(0.0);
    }

    //inputs the rotation object as radian conversion
    public Rotation2d getCurrentRotation() {
        return new Rotation2d(getAbsEncRadians());
    }

    public SwerveModuleState getModuleState() {
        double velocity = Conversions.falconToMPS(inputs.driveMtrVelocity, DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, DriveConstants.SDS_L2_GEAR_RATIO);
        
        return new SwerveModuleState(velocity, getCurrentRotation());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
    }
    
    public double getDriveDistanceMeters() {
        return  ((inputs.driveMtrSensorPosition / 2048)/ DriveConstants.SDS_L2_GEAR_RATIO) * DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE;
    }
}
