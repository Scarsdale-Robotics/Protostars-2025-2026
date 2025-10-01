package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class LocalizationSubsystem {
    static AprilTagDetection lastTagDetected;
    public static double getHeading(){
        return lastTagDetected.ftcPose.yaw;
    }



}
