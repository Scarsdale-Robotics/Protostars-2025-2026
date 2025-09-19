package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class RoadRunnerAuto extends LinearOpMode {
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;
    public SampleMecanumDrive drive;
    public Pose2d currentPose;
    double xDist = 0;
    double yDist = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        currentPose = new Pose2d(0,0,0);
        drive.setPoseEstimate(currentPose);
        this.drive = new SampleMecanumDrive(hardwareMap);
        Trajectory trajectory = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(100,100), robot.hardwareRobot.getHeading())
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(50,50), robot.hardwareRobot.getHeading())
                .build();
        waitForStart();
        while (opModeIsActive()) {
            currentPose = new Pose2d(robot.hardwareRobot.pinpoint.getPosX(DistanceUnit.INCH), robot.hardwareRobot.pinpoint.getPosY(DistanceUnit.INCH), robot.hardwareRobot.getHeading());
            drive.followTrajectory(trajectory);
        }
    }
    public boolean xInchRadius(int radius, AprilTagDetection target) {
        return target != null && target.ftcPose.range <= radius;
    }
    public void detectTags() {
        ArrayList<AprilTagDetection> detections = robot.cv.aprilTagProcessor.getDetections();
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                telemetry.addLine("AprilTag Detected.");
                telemetry.addData("ID", tag.id);
                telemetry.addData("X (Sideways offset)", tag.ftcPose.x);
                telemetry.addData("Y (Forward/Back Offset)", tag.ftcPose.y);
                telemetry.addData("Z", tag.ftcPose.z);
                telemetry.addData("Bearing", tag.ftcPose.bearing);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
                telemetry.addData("Range: ", tag.ftcPose.range);
                lastTagDetected = tag;
                break;
            }
        } else {
            lastTagDetected = null; // clear old tag when none detected
        }
    }
}
