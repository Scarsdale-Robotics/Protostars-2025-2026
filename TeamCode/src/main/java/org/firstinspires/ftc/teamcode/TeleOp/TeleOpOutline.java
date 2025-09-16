package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotSystem;

public class TeleOpOutline extends LinearOpMode {
    public double speed;
    public RobotSystem robot;
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


        }
    }
}
