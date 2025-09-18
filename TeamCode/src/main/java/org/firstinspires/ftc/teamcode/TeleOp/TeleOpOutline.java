package org.firstinspires.ftc.teamcode.TeleOp;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.ArrayList;

public class TeleOpOutline extends LinearOpMode {
    public double speed;
    public RobotSystem robot;

    public AprilTagDetection lastTagDetected;

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {
            if (gamepad1.a) {
                speed = 0.5;
            }
            else {
                speed = 0.25;
            }

            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            robot.drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);


            if (gamepad1.b){
                goalAlign(lastTagDetected);
            }

        }


    }
    public void aimTurn(double degrees) {
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
    }

    public void goalAlign(@NonNull AprilTagDetection target) {
        if (target.id == 20) {// 24 if on red team

            if (target.ftcPose.bearing != 0) {
                aimTurn(-1 * target.ftcPose.bearing);
            }
            if (target.ftcPose.x != 0) {
                aimTurn(-1 * target.ftcPose.x);
            }


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


}

