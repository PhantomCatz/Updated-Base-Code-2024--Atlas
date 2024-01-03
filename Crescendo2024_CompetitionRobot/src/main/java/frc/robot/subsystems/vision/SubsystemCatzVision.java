package frc.robot.subsystems.vision;
    
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.VisionConstants;

public class SubsystemCatzVision extends SubsystemBase {

    private static SubsystemCatzVision instance = null;

    private final VisionIO[] cameras;
    private final VisionIOInputsAutoLogged[] inputs;

    private final List<SubsystemCatzVision.PoseAndTimestamp> results = new ArrayList<>(); //in a list to account for multiple cameras

    private int acceptableTagID;
    private boolean useSingleTag = false;

    //constructor for vision subsystem that creates new vision input objects for each camera set in the singleton implementation
    private SubsystemCatzVision(VisionIO[] cameras) {
        this.cameras = cameras;
        inputs = new VisionIOInputsAutoLogged[cameras.length];
    }

    @Override
    public void periodic() {
        Logger.recordOutput("useSingleTag", useSingleTag); //set by driverstation

        // clear results from last periodic
        results.clear();
        
        //for every limlight camera process vision with according logic
        for (int i = 0; i < inputs.length; i++) {
            // update and process new inputs for camera
            cameras[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + cameras[i].getName() + "/Inputs", inputs[i]);

            //checks for when to process vision
            if (inputs[i].hasTarget
                    && inputs[i].isNewVisionPose
                    && !DriverStation.isAutonomous()
                    && inputs[i].maxDistance < VisionConstants.LOWEST_DISTANCE) 
            {
                if (useSingleTag) {
                    if (inputs[i].singleIDUsed == acceptableTagID) {
                        processVision(i);
                    }
                } 
                else {
                    processVision(i);
                }

            }
        }

        //Logging
        Logger.recordOutput("Vision/ResultCount", results.size());
    }

    public void processVision(int cameraNum) {
        // create a new pose based off the new inputs
        Pose2d currentPose = new Pose2d(inputs[cameraNum].x, 
                                        inputs[cameraNum].y, 
                                        new Rotation2d(inputs[cameraNum].rotation));

        //log data
        Logger.recordOutput(cameras[cameraNum].getName() + " pose", currentPose);

        // add the new pose to a list
        results.add(new PoseAndTimestamp(currentPose, inputs[cameraNum].timestamp));
    }

    //Returns the last recorded pose in a list
    public List<SubsystemCatzVision.PoseAndTimestamp> getVisionOdometry() {
        return results;
    }

    //Inner class to record a pose and its timestamp
    public static class PoseAndTimestamp {
        Pose2d pose;
        double timestamp;

        public PoseAndTimestamp(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }

    //access method for determining whether to use multiple tags for pose estimation
    public void setUseSingleTag(boolean useSingleTag) {
        setUseSingleTag(useSingleTag, 0);
    }

    public void setUseSingleTag(boolean useSingleTag, int acceptableTagID) {
        this.useSingleTag = useSingleTag;
        this.acceptableTagID = acceptableTagID;
    }

    //for photonvision, tbd can this be depreciated
    public void setReferencePose(Pose2d pose) {
        for (VisionIO io : cameras) {
            io.setReferencePose(pose);
        }
    }

    public double getMinDistance(int camera) {
        return inputs[camera].minDistance;
    }

    /**
    * singleton implenentation of vision
    * Any new cameras should be declared here
    */
    public static SubsystemCatzVision getInstance()
    {
        if(instance == null) {
            instance = new SubsystemCatzVision(new VisionIO[] {
                new VisionIOLimeLight("limelight", CatzConstants.VisionConstants.LIMELIGHT_OFFSET)
            });
        }
        return instance;
    }


}
