package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class SimpleAuto extends LinearOpMode {
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        waitForStart();
        while (opModeIsActive()) {
            turn(30);   // turn left 30 degrees
            sleep(200);
            driveToTag(lastTagDetected, 100, 100);
            //rely on odometry here - three sequences based off of motif
            String sequence = robot.decode(lastTagDetected);
            switch (sequence) {
                case "GPP":
                    break;
                case "PGP":
                    break;
                case "PPG":
                    break;
            }
            //do something involving color detection and odometry above
        }
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
    //TODO: tweak coordinates for actual placement in front of apriltag
    public void driveToTag(AprilTagDetection target, int xCoordinate, int yCoordinate) {

    }

    public void turn(int degrees) {
        PIDController controller = new PIDController(0.02, 0, 0.001);  // tune these
        controller.setTolerance(1);

        double targetAngle = robot.hardwareRobot.getHeading() + degrees;

        while (opModeIsActive() && !controller.atSetPoint()) {
            double power = controller.calculate(robot.hardwareRobot.getHeading(), targetAngle);
            robot.drive.driveRobotCentricPowers(0, 0, power);
            telemetry.addData("Heading", robot.hardwareRobot.getHeading());
            telemetry.addData("Power", power);
            telemetry.update();
        }
        robot.drive.driveRobotCentricPowers(0, 0, 0);
    }


    public boolean xInchRadius(int radius, AprilTagDetection target) {
        return target != null && target.ftcPose.range <= radius;
    }
    public void sequenceOne() {

    }
    public void sequenceTwo() {

    }
    public void sequenceThree() {

    }
}