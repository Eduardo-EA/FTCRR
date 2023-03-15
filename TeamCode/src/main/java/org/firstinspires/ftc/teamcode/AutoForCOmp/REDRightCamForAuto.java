package org.firstinspires.ftc.teamcode.AutoForCOmp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoForCOmp.CamTune.Section1;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;

@Autonomous
public class REDRightCamForAuto extends LinearOpMode
{

    Servo RightServo;
    Servo LeftServo;
    DcMotor LiftMotor;



    static final double COUNTS_PER_MOTOR_REV = 3895.9;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double LiftSpeed = 1;

    private final ElapsedTime runtime = new ElapsedTime();

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

     // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    //Section 1
    public static double FORWARD1 = 1;  //in Inches
    public static double STRAFELEFT1 = 10;
    public static double FORWARD2 = 6.3;
    public static double TURN1 = -1.6;
    public static double FORWARD3 = 1.8;
    public static double TURN2 = -7.5;



    @Override
    public void runOpMode()
    {

        RightServo = hardwareMap.get(Servo.class,"RightServo");
        LeftServo = hardwareMap.get(Servo.class,"LeftServo");
        RightServo.setPosition(.55);
        LeftServo.setPosition(.48);

        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        LiftMotor.setPower(LiftSpeed);
        sleep(500);
        LiftMotor.setPower(0);

        Trajectory Part1 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD1)
                .build();

        Trajectory Part1_1 = drive.trajectoryBuilder(Part1.end())
                .strafeLeft(STRAFELEFT1)
                .build();
        Trajectory Part1_2 = drive.trajectoryBuilder(Part1_1.end())
                .forward(FORWARD2)
                .build();

        TrajectorySequence turn1 = drive.trajectorySequenceBuilder(Part1_2.end())
                .turn(Math.toRadians(TURN1))
                .build();
        TrajectorySequence turn2 = drive.trajectorySequenceBuilder(turn1.end())
                .turn(Math.toRadians(TURN2))
                .build();


       Trajectory Part2 = drive.trajectoryBuilder(turn2.end())
               .forward(FORWARD3)
                .build();
      Trajectory Part3 = drive.trajectoryBuilder(Part2.end())
              .forward(-1.8)
              .build();
      TrajectorySequence turn3 = drive.trajectorySequenceBuilder(Part3.end())
              .turn(Math.toRadians(-7.5)) //This Will probably need changing
              .build();
      Trajectory Part4 = drive.trajectoryBuilder(turn3.end())
              .forward(5) //will need to be change
              .build();
        Trajectory Part5 = drive.trajectoryBuilder(turn3.end())
                .forward(10) //will need to be change
                .build();

        if (isStopRequested()) return;



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

//rep wait for start
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }



        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        //left 1 middle 2 Right 3
        /* Actually do something useful */
        if(tagOfInterest.id == LEFT){ //1
            //trajectory


            drive.followTrajectory(Part1);
            drive.followTrajectory(Part1_1);
           // drive.followTrajectory(Part1_2);
           // drive.followTrajectorySequence(turn1);

          //  LiftMotor.setPower(LiftSpeed);
          //  sleep(4800);
          //  LiftMotor.setPower(0);

           // drive.followTrajectorySequence(turn2);
           // drive.followTrajectory(Part2);
          //  drive.followTrajectory(Part3);

           // LiftMotor.setPower(-1);
            //sleep(4800);
           // LiftMotor.setPower(0);

           // drive.followTrajectorySequence(turn3);
           // drive.followTrajectory(Part4);








        }else if(tagOfInterest.id == MIDDLE){ //2
            drive.followTrajectory(Part1);
            drive.followTrajectory(Part1_1);
            drive.followTrajectory(Part1_2);
            drive.followTrajectorySequence(turn1);

            LiftMotor.setPower(LiftSpeed);
            sleep(3000);
            LiftMotor.setPower(0);

            drive.followTrajectorySequence(turn2);
            drive.followTrajectory(Part2);
            drive.followTrajectory(Part3);

            LiftMotor.setPower(-1);
            sleep(3000);
            LiftMotor.setPower(0);

            drive.followTrajectorySequence(turn3);

        }else if (tagOfInterest.id == RIGHT){ //3
            drive.followTrajectory(Part1);
            drive.followTrajectory(Part1_1);
            drive.followTrajectory(Part1_2);
            drive.followTrajectorySequence(turn1);

            LiftMotor.setPower(LiftSpeed);
            sleep(3000);
            LiftMotor.setPower(0);

            drive.followTrajectorySequence(turn2);
            drive.followTrajectory(Part2);
            drive.followTrajectory(Part3);

            LiftMotor.setPower(-1);
            sleep(3000);
            LiftMotor.setPower(0);

            drive.followTrajectorySequence(turn3);
            drive.followTrajectory(Part5);




        }






    }
/*
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

 */

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}