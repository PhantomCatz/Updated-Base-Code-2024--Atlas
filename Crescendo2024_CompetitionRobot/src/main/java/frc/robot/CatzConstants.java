package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
/***
 * CatzConstants
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * This class is where reusable constants are defined
 ***/
public final class CatzConstants {
  public static final boolean tuningMode = false;
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

 public static final class OIConstants {

  public static final int XBOX_DRV_PORT = 0;
  public static final int XBOX_AUX_PORT = 1;

  public static final int kDriverYAxis = 1;
  public static final int kDriverXAxis = 0;
  public static final int kDriverRotAxis = 4;
  public static final int kDriverFieldOrientedButtonIdx = 1;

  public static final double kDeadband = 0.1;
  }

  public static final class VisionConstants {
    public static final double LOWEST_DISTANCE = Units.feetToMeters(10.0);
    
    public static final Transform3d LIMELIGHT_OFFSET = new Transform3d(0.0, 0.0, 0.0, null); //tbd need to understand how these classese work transform3d vs translation3d
  }

  public static final class TrajectoryConstants {
    public static final double ALLOWABLE_POSE_ERROR = 0.05;
    public static final double ALLOWABLE_ROTATION_ERROR = Math.toRadians(2);
  }

  //--------------------------------------Drivetrain-------------------------------
  public static final class DriveConstants {

    public static final double LT_FRNT_OFFSET = 0.5112305378; //this one changed
    public static final double LT_BACK_OFFSET = 0.5446386386;
    public static final double RT_BACK_OFFSET = 0.7591109064;
    public static final double RT_FRNT_OFFSET = 0.5363121009;

    public static final int LT_FRNT_DRIVE_ID = 1;
    public static final int LT_BACK_DRIVE_ID = 3;//TBD put in constants
    public static final int RT_BACK_DRIVE_ID = 22;
    public static final int RT_FRNT_DRIVE_ID = 7;
    
    public static final int LT_FRNT_STEER_ID = 2;
    public static final int LT_BACK_STEER_ID = 4;
    public static final int RT_BACK_STEER_ID = 6;
    public static final int RT_FRNT_STEER_ID = 8;

    public static final int LT_FRNT_ENC_PORT = 9;
    public static final int LT_BACK_ENC_PORT = 6;
    public static final int RT_BACK_ENC_PORT = 7;
    public static final int RT_FRNT_ENC_PORT = 8;

    public static final int     CURRENT_LIMIT_AMPS            = 55;
    public static final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    public static final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    public static final boolean ENABLE_CURRENT_LIMIT          = true;

    public static final int     STEER_CURRENT_LIMIT_AMPS      = 30;
    public static final double  NEUTRAL_TO_FULL_SECONDS       = 0.1;
    public static final double  VEL_FF                        = 1.5;

    public static final Pose2d initPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
    private static final double MODULE_DISTANCE_FROM_CENTER = 0.298;

    public static final double ESTIMATION_COEFFICIENT = 0.025;

    //not following the original coordinate system since the robot coordinate system is inverted
    private static final Translation2d SWERVE_LEFT_FRONT_LOCATION  = new Translation2d(MODULE_DISTANCE_FROM_CENTER, MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d SWERVE_LEFT_BACK_LOCATION   = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d SWERVE_RIGHT_BACK_LOCATION  = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER);
    
    // calculates the orientation and speed of individual swerve modules when given the motion of the whole robot
    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        SWERVE_LEFT_FRONT_LOCATION,
        SWERVE_LEFT_BACK_LOCATION,
        SWERVE_RIGHT_BACK_LOCATION,
        SWERVE_RIGHT_FRONT_LOCATION
    );
    
    public static final double MAX_SPEED = 3.0; // meters per second
    public static final double MAX_ANGSPEED_RAD_PER_SEC = 2.0; // radians per second
    public static final double MAX_SPEED_DESATURATION = MAX_SPEED + MAX_ANGSPEED_RAD_PER_SEC * MODULE_DISTANCE_FROM_CENTER * Math.sqrt(2);

    public static final double SDS_L1_GEAR_RATIO = 8.14;       //SDS mk4i L1 ratio reduction
    public static final double SDS_L2_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio reduction
    
    public static final double DRVTRAIN_WHEEL_DIAMETER_METERS = 0.095;
    public static final double DRVTRAIN_WHEEL_CIRCUMFERENCE   = (Math.PI * DRVTRAIN_WHEEL_DIAMETER_METERS);

    //uses a trapezoidal velocity/time graph enforced with a PID loop
    private static ProfiledPIDController autoTurnPIDController
            = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGSPEED_RAD_PER_SEC, MAX_ANGSPEED_RAD_PER_SEC));
        //TBD need to validated
    static{
        autoTurnPIDController.enableContinuousInput(-Math.PI, Math.PI); //offset clamped between these two values
        autoTurnPIDController.setTolerance(Math.toRadians(0.1)); //tolerable error
    }
    
    //TBD need to validated
    // calculates target chassis motion when given current position and desired trajectory
    public static final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(2, 0, 0), // PID values for x offset
        new PIDController(2, 0, 0), // PID values for y offset
        autoTurnPIDController // PID values for orientation offset
    );

    // calculates target chassis motion when given current position and desired trajectory
    public static final PPHolonomicDriveController ppholonomicDriveController = new PPHolonomicDriveController(
        new PIDConstants(0.35, 0, 0), // PID values for x offset
        new PIDConstants(0.35, 0, 0), // PID values for rotation 
        MAX_SPEED,
        MODULE_DISTANCE_FROM_CENTER
    );

    public static final HolonomicPathFollowerConfig pathFollowingConfig = new HolonomicPathFollowerConfig( 
        MAX_SPEED, // Max module speed, in m/s
        MODULE_DISTANCE_FROM_CENTER, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig()); // Default path replanning config. See the API for the options here
  }

}
