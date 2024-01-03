package frc.robot;

import java.sql.Driver;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;


public class CatzAutonomous {

    
    private static PathPlannerPath driveStraighFullTurn = PathPlannerPath.fromPathFile("DriveStraightFullTurn");
    private static PathPlannerPath feildSideDriveBack = PathPlannerPath.fromPathFile("FeildSideDriveBack");

    public static LoggedDashboardChooser<DriverStation.Alliance> chosenAllianceColor = new LoggedDashboardChooser<>("alliance selector");
    private static LoggedDashboardChooser<AutoModes> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    private enum AutoModes {
        TEST,
        DRIVE_STRAIGHT,
        TRANSLATE_DRIVE_STRAIGHT,
    }

    public CatzAutonomous()
    {
        chosenAllianceColor.addDefaultOption("Blue Alliance", DriverStation.Alliance.Blue);
        chosenAllianceColor.addOption       ("Red Alliance",  DriverStation.Alliance.Red);

        autoChooser.addDefaultOption("Do Nothing", null);

        autoChooser.addOption       ("TEST PATH",  AutoModes.TEST);
        autoChooser.addOption       ("Drivestright", AutoModes.DRIVE_STRAIGHT);
        autoChooser.addOption       ("TranslateDriveStraight", AutoModes.TRANSLATE_DRIVE_STRAIGHT);
    }

    public Command getCommand()
    {
        SubsystemCatzDrivetrain.getInstance().resetForAutonomous();

        switch(autoChooser.get())
        {
            case TEST: return testPath2();
            case DRIVE_STRAIGHT: return driveStraight();
            default: 
            return new InstantCommand();
        }
    }

    public Command testPath() {
        return new SequentialCommandGroup();
    }

    public Command testPath2() {
        return new SequentialCommandGroup();
    }

    public Command driveStraight() {
        return new SequentialCommandGroup();
    }

    public Command startWall()
    {
        return new SequentialCommandGroup();
    }

    public Command startField()
    {
        return new SequentialCommandGroup();
    }

    public Command startCenter()
    {
        return new SequentialCommandGroup();
    }

}