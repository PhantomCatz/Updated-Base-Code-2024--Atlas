package frc.robot;

import java.sql.Driver;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.google.flatbuffers.Constants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.commands.DriveCmds.PPTrajectoryFollowingCmd;
import frc.robot.commands.DriveCmds.Trajectory.TrajectoryFollowingCmd;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;


public class CatzAutonomous {

    private SubsystemCatzDrivetrain m_driveTrain = SubsystemCatzDrivetrain.getInstance();
    private static PathPlannerPath driveStraighFullTurn = PathPlannerPath.fromPathFile("DriveStraightFullTurn");
    private static PathPlannerPath feildSideDriveBack = PathPlannerPath.fromPathFile("FeildSideDriveBack");

    private static LoggedDashboardChooser<DriverStation.Alliance> chosenAllianceColor = new LoggedDashboardChooser<>("alliance selector");
    private static LoggedDashboardChooser<AutoModes> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    private enum AutoModes {
        TEST,
        DRIVE_STRAIGHT,
        TRANSLATE_DRIVE_STRAIGHT,
    }

    public CatzAutonomous() {
        chosenAllianceColor.addDefaultOption("Blue Alliance", DriverStation.Alliance.Blue);
        chosenAllianceColor.addOption       ("Red Alliance",  DriverStation.Alliance.Red);

        autoChooser.addDefaultOption("Do Nothing", null);

        autoChooser.addOption       ("TEST PATH",  AutoModes.TEST);
        autoChooser.addOption       ("Drivestright", AutoModes.DRIVE_STRAIGHT);
        autoChooser.addOption       ("TranslateDriveStraight", AutoModes.TRANSLATE_DRIVE_STRAIGHT);
    }

    //configured dashboard
    public Command getCommand() {
        m_driveTrain.resetForAutonomous();

        switch(autoChooser.get()) {
            case TEST: return testPath2();
            case DRIVE_STRAIGHT: return driveStraight();
            default: 
            return new InstantCommand();
        }
    }

    //---------------------------------------------------------Autonmous Autos---------------------------------------------------------
    public Command testPath() {
        return new SequentialCommandGroup();
    }

    public Command testPath2() {
        return new SequentialCommandGroup();
    }

    public Command driveStraight() {
    return new PPTrajectoryFollowingCmd(driveStraighFullTurn);
    }
    
    //---------------------------------------------------------Trajectories/Swervepathing---------------------------------------------------------

    public Command flyTrajectoryOne() {
        // Create a list of bezier points from poses. Each pose represents one waypoint. 
        // The rotation component of the pose should be the direction of travel.
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
        );
        PathConstraints pathConstraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path. 
        GoalEndState endState = new GoalEndState(0.0, Rotation2d.fromDegrees(-90)); // Goal end state. You can set a holonomic rotation here.

        return new PPTrajectoryFollowingCmd(bezierPoints, pathConstraints, endState);
    }

    //Automatic pathfinding command
    public Command autoFindPathOne() {
                // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0, 
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        // See the "Follow a single path" example for more info on what gets passed here
        return AutoBuilder.pathfindThenFollowPath(
                driveStraighFullTurn,
                constraints,
                3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
    }

}