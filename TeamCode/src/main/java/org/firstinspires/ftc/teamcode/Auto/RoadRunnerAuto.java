package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
//TODO: implement pose tracking with control hub imu instead of pinpoint imu.
public class RoadRunnerAuto extends LinearOpMode {
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;
    public SampleMecanumDrive drive;
    public Pose2d startPose;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotSystem(hardwareMap, this);
        drive = new SampleMecanumDrive(hardwareMap, robot);
        robot.hardwareRobot.initOdom();
        robot.hardwareRobot.setImu();
        startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(100, 100), 0)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .splineTo(new Vector2d(50, 50), 0)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectoryAsync(trajectory1);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            detectTags();
            telemetry.addData("pose", drive.getPoseEstimate());
            telemetry.update();
        }

        drive.followTrajectoryAsync(trajectory2);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            detectTags();
            telemetry.addData("pose", drive.getPoseEstimate());
            telemetry.update();
        }
    }

    public boolean xInchRadius(int radius, AprilTagDetection target) {
        return target != null && target.ftcPose.range <= radius;
    }

    public void detectTags() {
        ArrayList<AprilTagDetection> detections = robot.cv.aprilTagProcessor.getDetections();
        if (detections != null && !detections.isEmpty()) {
            lastTagDetected = detections.get(0);
            telemetry.addLine("AprilTag Detected.");
            telemetry.addData("ID", lastTagDetected.id);
            telemetry.addData("X (Sideways offset)", lastTagDetected.ftcPose.x);
            telemetry.addData("Y (Forward/Back Offset)", lastTagDetected.ftcPose.y);
            telemetry.addData("Z", lastTagDetected.ftcPose.z);
            telemetry.addData("Bearing", lastTagDetected.ftcPose.bearing);
            telemetry.addData("Yaw", lastTagDetected.ftcPose.yaw);
            telemetry.addData("Range", lastTagDetected.ftcPose.range);
            telemetry.update();
        } else {
            lastTagDetected = null; // clear old tag when none detected
        }
    }
}

