package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.controller.PIDController;
import com.fasterxml.jackson.databind.JsonSerializable;
import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.source.util.TaskListener;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.security.Policy;
import java.util.ArrayList;
import java.util.Arrays;

//TODO: TUNE PID, CENTRIPETAL, ALL CONSTANTS, AND EXPERIMENT WITH INTERPOLATION.
//TODO: download ftcdashboard and tune constants with drive test
//TODO: finish path implementation: fix preload path
public class PedroPathingAuto extends LinearOpMode {
    public RobotSystem robot = new RobotSystem(hardwareMap, this);
    public PathChain detectionPathChain;
    public PathChain pickupPathChain1;
    public PathChain pickupPathChain2;
    public PathChain pickupPathChain3;
    public PathChain scorePathChain;
    public PathChain scorePreloadPathChain;
    public Follower follower;
    public Timer pathTimer, opmodeTimer;
    public int pathState;
    public final Pose startPose = new Pose(0,0,0);
    public final Pose apTag1 = new Pose(70,50, startPose.getHeading());
    //quad bezier curve, rest linear w little to no interpolation
    public Pose pickup = new Pose(40, 84, 180);
    public Pose step1 = new Pose(-20, 30, 90);
    public Pose step1a = new Pose(-19, 30,90);
    public Pose step1b = new Pose(-18, 30, 90);
    public Pose step1c = new Pose(-17, 30, 90);
    // quadratic bezier curve for this step
    public Pose alignGoal = new Pose(30, 125, 143);
    public PathChain scorePreloadPathChainPtTwo;
    public Pose alignGoal2 = new Pose(30, 120, 143);
    public AprilTagDetection lastTagDetected;
    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        setPathState(-1);
        follower.setStartingPose(startPose);
        buildPaths();
        while (opModeIsActive()) {
            detectTags();
            follower.update();
            autonomousPathUpdate();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
    public boolean xInchRadius(int radius, AprilTagDetection target) {
        return target.ftcPose.range <= radius;
    }
    public void buildPaths() {
        this.scorePreloadPathChain = follower.pathBuilder()
                .addPath(new BezierLine(apTag1, alignGoal2))
                .setConstantHeadingInterpolation(90)
                //shoot
                .build();
        this.scorePreloadPathChainPtTwo = follower.pathBuilder()
                .addPath(new BezierLine(alignGoal2, apTag1))
                .setConstantHeadingInterpolation(90)
                .build();
        this.detectionPathChain = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(startPose, new Pose(70,10,robot.hardwareRobot.getHeading()), apTag1)))
                .setLinearHeadingInterpolation(0,90)
                .build();
        //how to use curve?
        this.pickupPathChain1 = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(apTag1, new Pose(70,100,90), pickup)))
                .setLinearHeadingInterpolation(robot.hardwareRobot.getHeading(), 180)
                .build();
        this.pickupPathChain2 = follower.pathBuilder()
                .addPath(new BezierLine(apTag1, new Pose(40, 61, Math.toRadians(0))))
                .setLinearHeadingInterpolation(robot.hardwareRobot.getHeading(), 180)
                .build();
        this.pickupPathChain3 = follower.pathBuilder()
                .addPath(new BezierLine(apTag1, new Pose(40, 35, Math.toRadians(0))))
                .setLinearHeadingInterpolation(robot.hardwareRobot.getHeading(), 180)
                .build();
        this.scorePathChain = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(new Pose(40,35,0), new Pose(70,80, 0), alignGoal)))
                .setLinearHeadingInterpolation(robot.hardwareRobot.getHeading(), 143)
                .build();
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
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) follower.followPath(scorePreloadPathChain);
                //shoot
                if (!follower.isBusy()) follower.followPath(scorePreloadPathChainPtTwo);
                setPathState(1);
            case -1:
                if (!follower.isBusy()) follower.followPath(detectionPathChain);
                setPathState(0);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    follower.setPose(getRobotPoseFromCamera(lastTagDetected));
                    if (robot.decode(lastTagDetected).equals("PPG")) follower.followPath(pickupPathChain3);
                    else if (robot.decode(lastTagDetected).equals("PGP")) follower.followPath(pickupPathChain2);
                    else follower.followPath(pickupPathChain1);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    follower.followPath(scorePathChain);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-2);
                }
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    private Pose getRobotPoseFromCamera(AprilTagDetection tag) {
        return new Pose(tag.robotPose.getPosition().x, tag.robotPose.getPosition().y, robot.hardwareRobot.getHeading(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
