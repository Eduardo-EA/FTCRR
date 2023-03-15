package org.firstinspires.ftc.teamcode.TrashBut;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Disabled
@Autonomous(group = "drive")
public class TestAuto extends LinearOpMode {


    Servo RightServo;
    Servo LeftServo;
    DcMotor LiftMotor;

    static final double COUNTS_PER_MOTOR_REV = 3895.9;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    double LiftSpeed = 1;
    private final ElapsedTime runtime = new ElapsedTime();


    public static double FORWARD1 = 1;  //in Inches
    public static double STRAFELEFT1 = 22;
    public static double FORWARD2 = 25;
    public static double TURN1 = .8;
    public static double FORWARD3 = 5;
    public static double WAIT1 = 1;   //adjust for later
    public static double FORWARD4 = -5;
    public static double TURN2 = -.8;
    public static double FORWARD5 = 24.32;
    public static double TURN3 = -1.58;
    public static double FORWARD6 = 43;
    public static double WAIT2 = 1; //adjust for later
    public static double FORWARD7 = -20.5;
    public static double TURN4 = -.8;
    public static double FORWARD8 = 5;
    public static double WAIT3 = 1; //adjust for later
    public static double FORWARD9 = -5;
    public static double TURN5 = .8;
    public static double FORWARD10 = 20.5;
    public static double WAIT4 = 1; //adjust for later
    public static double FORWARD11 = -20.5;
    public static double TURN6 = .8;
    public static double FORWARD12 = 4;
    public static double WAIT5 = 1; //adjust for later
    public static double FORWARD13 = -4;
    public static double TURN7 = -.8;
    public static double FORWARD14 = -24;
    public static double STRAFERIGHT1 = 15;
    public static double TURN8 = -1.5;
    public static double FORWARD15 = 10;
    public static double STRAFELEFT2 = 1;
    public static double TURN9 = -.1;


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        RightServo = hardwareMap.get(Servo.class,"RightServo");
        LeftServo = hardwareMap.get(Servo.class,"LeftServo");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Opens claw
        RightServo.setPosition(.35);
        LeftServo.setPosition(.65);

        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Trajectory Part1 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD1)
                .strafeLeft(STRAFELEFT1)
                .forward(FORWARD2)
                .build();
        Trajectory Part2 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD3)
                .build();
      /*  TrajectorySequence Wait1 = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(WAIT1)
                .build();
        Trajectory Part3 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD4)
                .build();
        Trajectory Part4 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD5)
                .build();
        Trajectory Part5 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD6)
                .build();
        TrajectorySequence Wait2 = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(WAIT2)
                .build();
        Trajectory Part6 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD7)
                .build();
        Trajectory Part7 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD8)
                .build();
        TrajectorySequence wait3 = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(WAIT3)
                .build();
        Trajectory Part8 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD9)
                .build();
        Trajectory Part9 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD10)
                .build();
        TrajectorySequence wait4 = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(WAIT4)
                .build();
        Trajectory Part10 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD11)
                .build();
        Trajectory Part11 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD12)
                .build();
        TrajectorySequence wait5 = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(WAIT5)
                .build();
        Trajectory Part12 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD13)
                .build();
        Trajectory Part13 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD14)
                .strafeRight(STRAFERIGHT1)
                .build();
        Trajectory Part14 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD15)
                .strafeRight(STRAFELEFT2)
                .build();
        TrajectorySequence turn1 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(TURN1))
                .build();
        TrajectorySequence turn2 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(TURN2))
                .build();
        TrajectorySequence turn3 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(TURN3))
                .build();
        TrajectorySequence turn4 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(TURN4))
                .build();
        TrajectorySequence turn5 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(TURN5))
                .build();
        TrajectorySequence turn6 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(TURN6))
                .build();
        TrajectorySequence turn7 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(TURN7))
                .build();
        TrajectorySequence turn8 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(TURN8))
                .build();
        TrajectorySequence turn9 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(TURN9))
                .build();

       */



        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(Part1);

       /* drive.followTrajectorySequence(turn1);
        //  drive.turn(Math.toRadians(TURN1));
        drive.followTrajectory(Part2);

        drive.followTrajectorySequence(Wait1);

        encoderDrive(10);

        drive.followTrajectory(Part3);

        // drive.turn(Math.toRadians(TURN2));
        drive.followTrajectorySequence(turn2);
        drive.followTrajectory(Part4);

        //  drive.turn(Math.toRadians(TURN3));
        drive.followTrajectorySequence(turn3);

        drive.followTrajectory(Part5);
        drive.followTrajectorySequence(Wait2);

        //drive.turn(Math.toRadians(TURN4));
        drive.followTrajectorySequence(turn4);


        drive.followTrajectory(Part6);

        // drive.turn(Math.toRadians(TURN4));
        drive.followTrajectorySequence(turn4);

        drive.followTrajectory(Part7);
        drive.followTrajectorySequence(wait3);
        drive.followTrajectory(Part8);

        // drive.turn(Math.toRadians(TURN5));
        drive.followTrajectorySequence(turn5);

        drive.followTrajectory(Part9);
        drive.followTrajectorySequence(wait4);
        drive.followTrajectory(Part10);


        // drive.turn(Math.toRadians(TURN6));
        drive.followTrajectorySequence(turn6);

        drive.followTrajectory(Part11);
        drive.followTrajectorySequence(wait5);
        drive.followTrajectory(Part12);

        //  drive.turn(Math.toRadians(TURN7));
        drive.followTrajectorySequence(turn7);

        drive.followTrajectory(Part13);


        // drive.turn(Math.toRadians(TURN8));
        drive.followTrajectorySequence(turn8);

        drive.followTrajectory(Part14);


        // drive.turn(Math.toRadians(TURN9));
        drive.followTrajectorySequence(turn9);

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        */

        Pose2d poseEstimate = drive.getPoseEstimate();

        while (!isStopRequested() && opModeIsActive()) ;
    }
    private void encoderDrive(double LiftMotorInches){
        int newLiftMotorTarget;

        if (opModeIsActive()) {
            newLiftMotorTarget = LiftMotor.getCurrentPosition() + (int)(LiftMotorInches * COUNTS_PER_INCH);

            LiftMotor.setTargetPosition(newLiftMotorTarget);
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            LiftMotor.setPower(Math.abs(LiftSpeed));

        }
    }
}
