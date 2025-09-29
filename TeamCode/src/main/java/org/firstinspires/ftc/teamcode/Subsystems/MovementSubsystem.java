package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class MovementSubsystem extends SubsystemBase {

    public Motor frontLeftMotor;
    public Motor frontRightMotor;
    public Motor backLeftMotor;
    public Motor backRightMotor;
    public MecanumDrive motorControl;

    double speed;


    public void driveRobotCentric (double forward, double strafe, double turn) {


        frontLeftMotor.setTargetDistance(forward + strafe);
        frontRightMotor.setTargetDistance(forward - strafe);
        backLeftMotor.setTargetDistance(forward - strafe);
        backRightMotor.setTargetDistance(forward + strafe);


    }




}
