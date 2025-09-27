package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//TODO: make macros
public class RoadRunnerTeleOp extends LinearOpMode {
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;
    public SampleMecanumDrive drive;
    public Pose2d startPose;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        robot.hardwareRobot.setImu();
        robot.hardwareRobot.initOdom();
        this.startPose = new Pose2d(0,0,0);
        this.drive = new SampleMecanumDrive(hardwareMap, robot);
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
