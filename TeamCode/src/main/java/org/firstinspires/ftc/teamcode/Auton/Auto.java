package org.firstinspires.ftc.teamcode.Auton;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.Subsystems.MovementSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.ArrayList;

public class Auto extends LinearOpMode {

    public double speed;
    public RobotSystem robot;
    public AprilTagDetection lastTagDetected;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotSystem(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            robot.drive.driveRobotCentric(0, 1, 0);

        }

    }

}

