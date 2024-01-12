package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.commands.DriveCmds.TeleopDriveCmd;
import frc.robot.subsystems.SubsystemCatzArm.SubsystemCatzArm;
import frc.robot.subsystems.SubsystemCatzClimb.SubsystemCatzClimb;
import frc.robot.subsystems.SubsystemCatzShooter.SubsystemCatzShooter;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.vision.SubsystemCatzVision;

/**
 * RobotContainer
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * This class is how the command scheduler(robot.java replacment) is configured
 * Configures:
 * -xbox controller triggers
 * -default commands
 * -instanciated mechanisms using singleton implementation
 * -sets up autonomous from CatzAtutonomouschooser
 */
 public class RobotContainer {
    
    //subsystems
    private SubsystemCatzDrivetrain driveTrain; 
    private SubsystemCatzVision vision;
    private SubsystemCatzShooter shooter;
    private SubsystemCatzClimb climb;
    private SubsystemCatzArm arm;

    private CatzAutonomous auton = new CatzAutonomous();

    //xbox controller
    private CommandXboxController xboxDrv;
    private CommandXboxController xboxAux;
 
       
   /** The container for the robot. Contains subsystems, OI devices, and commands. 
    *    -since multiple classes are referencing these mechansims, 
    *     mechanisms are instantiated inside mechanism class(singleton)
    */
   public RobotContainer() {
    //instantiate subsystems
     driveTrain = SubsystemCatzDrivetrain.getInstance(); 
     vision     = SubsystemCatzVision.getInstance();

     shooter    = SubsystemCatzShooter.getInstance();
     climb      = SubsystemCatzClimb.getInstance();
     arm        = SubsystemCatzArm.getInstance();

 
     xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT); 
     xboxAux = new CommandXboxController(OIConstants.XBOX_AUX_PORT);
 
     // Configure the trigger bindings and default cmds
     defaultCommands();
     configureBindings();
   }
 
   
   private void configureBindings() {
 
    xboxDrv.a().onTrue(auton.flyTrajectoryOne());
    xboxDrv.back().onTrue(driveTrain.toggleVisionEnableCommand());
    xboxDrv.start().onTrue(driveTrain.flipGyro());
    xboxDrv.b().onTrue(driveTrain.stopDriving()); //TBD need to add this back in TBD runs when disabled where?

    //shooter activation
    xboxDrv.x().onTrue(shooter.setShooterActive())
               .onFalse(shooter.setShooterDisabled());
 
   }

   //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
   private void defaultCommands() {  
      driveTrain.setDefaultCommand(new TeleopDriveCmd(()-> xboxDrv.getLeftX(),
                                                      ()-> xboxDrv.getLeftY(),
                                                      ()-> xboxDrv.getRightX(),
                                                      ()-> xboxDrv.getRightTriggerAxis(), 
                                                      ()-> xboxDrv.b().getAsBoolean()));
    
   }

   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
   public Command getAutonomousCommand() {
     return auton.getCommand();
   }
}
