package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class RoadRunnerTeleOp extends LinearOpMode {
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;
    public SampleMecanumDrive drive;
    public Pose2d startPose;
    public double speed;
    public boolean crPressed = false;
    public boolean sqPressed = false;
    boolean lastCrPressed = false;
    public boolean lastSqPressed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        this.speed = 0.7;
        this.robot = new RobotSystem(hardwareMap, this);
        robot.hardwareRobot.setImu();
        robot.hardwareRobot.initOdom();
        this.startPose = new Pose2d(0,0,0);
        this.drive = new SampleMecanumDrive(hardwareMap, robot);
        waitForStart();
        while (opModeIsActive()) {
            robot.hardwareRobot.pinpoint.update();
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            if (xInchRadius(10, lastTagDetected)) {
                speed = 0.5;
            } else {
                speed = 0.7;
            }
            robot.drive.driveFieldCentricPowers(strafe * speed, forward * speed, turn * speed, robot.hardwareRobot.getHeading());
            sqPressed = gamepad1.square;
            if (sqPressed && !lastSqPressed) {
                runTrajectory(new Vector2d(20,20), 0);
                //shoot x3
            }
            crPressed = gamepad1.circle;
            if (crPressed && !lastCrPressed) {
                runTrajectory(new Vector2d(40,40), 10);
                //hardcode rest of pickup sequence
            }
            detectTags();
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Forward", forward);
            telemetry.addData("Turn", turn);
            startPose = new Pose2d(robot.hardwareRobot.pinpoint.getPosX(DistanceUnit.INCH), robot.hardwareRobot.pinpoint.getPosY(DistanceUnit.INCH), robot.hardwareRobot.getHeading());
            lastSqPressed = sqPressed;
            lastCrPressed = crPressed;
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
    public void runTrajectory(Vector2d finalPos, int endTangent) {
        drive.setPoseEstimate(startPose);
        Trajectory trajectory = drive.trajectoryBuilder(startPose)
                .splineTo(finalPos, 0)
                .build();
        drive.followTrajectoryAsync(trajectory);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            telemetry.addData("Pose", drive.getPoseEstimate());
        }
    }
}
