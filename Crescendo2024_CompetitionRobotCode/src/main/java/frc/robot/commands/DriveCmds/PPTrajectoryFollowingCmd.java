package frc.robot.commands.DriveCmds;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.google.flatbuffers.Constants;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.CatzConstants.TrajectoryConstants;

import java.util.List;

import org.littletonrobotics.junction.Logger;

public class PPTrajectoryFollowingCmd extends Command {

    private PathPlannerTrajectory.State previousState;
    
    private final Timer timer = new Timer();
    private final PPHolonomicDriveController controller;
    private SubsystemCatzDrivetrain m_driveTrain = SubsystemCatzDrivetrain.getInstance();

    private PathPlannerTrajectory trajectory;

    /**
     * The auto balance on charge station command constructor.
     *
     * @param drivetrain The coordinator between the gyro and the swerve modules.
     * @param trajectory          The trajectory to follow.
     */
    public PPTrajectoryFollowingCmd(PathPlannerPath newPath) {
        this.trajectory = new PathPlannerTrajectory(
                                newPath, 
                                DriveConstants.
                                    swerveDriveKinematics.
                                        toChassisSpeeds(m_driveTrain.getModuleStates()),
                                m_driveTrain.getRotation2d());

        controller = DriveConstants.ppholonomicDriveController;
        addRequirements(m_driveTrain);
    }

    //Trajectories w/o path planner
    public PPTrajectoryFollowingCmd(List<Translation2d> bezierPoints, PathConstraints constraints, GoalEndState endRobotState) {

        PathPlannerPath newPath = new PathPlannerPath(bezierPoints, constraints, endRobotState);

        this.trajectory = new PathPlannerTrajectory(
                                newPath, 
                                DriveConstants.
                                    swerveDriveKinematics.
                                        toChassisSpeeds(m_driveTrain.getModuleStates()), 
                                m_driveTrain.getRotation2d());

        controller = DriveConstants.ppholonomicDriveController;
        addRequirements(m_driveTrain);
    }

    @Override
    public void initialize() {
        // Reset and begin timer
        timer.reset();
        timer.start();
        // Get initial state of path
        previousState = trajectory.getInitialState();
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();

        //Determine desired state based on where the robot should be at the current time in the path
        PathPlannerTrajectory.State goal = (PathPlannerTrajectory.State) trajectory.sample(currentTime);
        Pose2d currentPosition = m_driveTrain.getPose();

        // obtain target velocity based on current pose and desired state
        ChassisSpeeds chassisSpeeds = controller.calculateRobotRelativeSpeeds(currentPosition, goal);

        m_driveTrain.driveRobotWithCorrectedDynamics(chassisSpeeds);
        Logger.recordOutput("Desired Auto Pose", new Pose2d(goal.positionMeters, goal.heading));

        previousState = goal;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop(); // Stop timer
        m_driveTrain.stopDriving();
    }

    @Override
    public boolean isFinished() {
        // Finish command if the total time the path takes is over
        double currentPosX =        m_driveTrain.getPose().getX();
        double currentPosY =        m_driveTrain.getPose().getY();
        double currentRotation =    m_driveTrain.getPose().getRotation().getRadians();

        double desiredPosX =        trajectory.getEndState().positionMeters.getX();
        double desiredPosY =        trajectory.getEndState().positionMeters.getY();
        double desiredRotation =    trajectory.getEndState().positionMeters.getAngle().getRadians();

        double xError =        Math.abs(desiredPosX - currentPosX);
        double yError =        Math.abs(desiredPosY - currentPosY);
        double rotationError = Math.abs(desiredRotation - currentRotation);

        return (timer.get() > trajectory.getTotalTimeSeconds()) ||
               (xError < TrajectoryConstants.ALLOWABLE_POSE_ERROR && 
                yError < TrajectoryConstants.ALLOWABLE_POSE_ERROR && 
                rotationError < TrajectoryConstants.ALLOWABLE_ROTATION_ERROR) || 
                timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}