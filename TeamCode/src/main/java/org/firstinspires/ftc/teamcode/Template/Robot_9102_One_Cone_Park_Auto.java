package org.firstinspires.ftc.teamcode.Template;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoForCOmp.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Config
@Autonomous
public class Robot_9102_One_Cone_Park_Auto extends LinearOpMode
{
  //  private final ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.AutoForCOmp.AprilTagDetectionPipeline aprilTagDetectionPipeline;

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

 //   static final double FEET_PER_METER = 3.28084;

    double LiftSpeed = 1;

    Servo RightServo;
    Servo LeftServo;
     DcMotor LiftMotor;



    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        RightServo = hardwareMap.get(Servo.class,"RightServo");
        LeftServo = hardwareMap.get(Servo.class,"LeftServo");


        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");


        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        drive.setPoseEstimate(new Pose2d(35.87,-60.07, Math.toRadians(90)));


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


        TrajectorySequence park1 = drive.trajectorySequenceBuilder(new Pose2d(39.04, -63.21, Math.toRadians(90.00)))
                .addDisplacementMarker(() ->{
                    RightServo.setPosition(.35);
                    LeftServo.setPosition(.65);

                })

                .lineToLinearHeading(new Pose2d(34, -58.96, Math.toRadians(90.00))) // center on first square
                .lineToLinearHeading(new Pose2d(34.79, -10.77, Math.toRadians(90))) //goes forward to 3 square
                .addTemporalMarker(.1, () -> {
                    // This marker runs two seconds into the trajectory
                    LiftMotor.setPower(.2); // was -2



                })

                .lineToLinearHeading(new Pose2d(31.5, -6.5, Math.toRadians(138.24))) // turns torwards junction and move forward
                .waitSeconds(1)
                .addTemporalMarker(3, () -> {
                    // This marker runs two seconds into the trajectory
                    LiftMotor.setPower(0);



                })

                .addTemporalMarker(2.5, () -> {

                    LiftMotor.setPower(-.8);


                })


                .addTemporalMarker(3, () -> {

                    RightServo.setPosition(.55);
                    LeftServo.setPosition(.48);
                    LiftMotor.setPower(0);

                })

                .lineToLinearHeading(new Pose2d(34.64, -11.50, Math.toRadians(90))) //\.a
                .lineToLinearHeading(new Pose2d(14,-12.50, Math.toRadians(90)))


                .build();












        TrajectorySequence park2 = drive.trajectorySequenceBuilder(new Pose2d(39.04, -63.21, Math.toRadians(90.00)))
                .addDisplacementMarker(() ->{
                    RightServo.setPosition(.35);
                    LeftServo.setPosition(.65);

                })

                .lineToLinearHeading(new Pose2d(34, -58.96, Math.toRadians(90.00))) // center on first square
                .lineToLinearHeading(new Pose2d(34.79, -10.77, Math.toRadians(90))) //goes forward to 3 square
                .addTemporalMarker(.1, () -> {
                    // This marker runs two seconds into the trajectory
                    LiftMotor.setPower(.2); // was -2


                    // Run your action in here!
                })

                .lineToLinearHeading(new Pose2d(31.5, -6.5, Math.toRadians(138.24))) // turns torwards junction and move forward
                .waitSeconds(1)
                .addTemporalMarker(3, () -> {
                    // This marker runs two seconds into the trajectory
                    LiftMotor.setPower(0);


                    // Run your action in here!
                })
                //.waitSeconds(2)  cam commit
                .addTemporalMarker(2.5, () -> {

                    LiftMotor.setPower(-.8);


                })


                .addTemporalMarker(3, () -> {

                    RightServo.setPosition(.55);
                    LeftServo.setPosition(.48);
                    LiftMotor.setPower(0);

                })

                .lineToLinearHeading(new Pose2d(34.64, -12.50, Math.toRadians(90)))//\.a


                .build();























        TrajectorySequence park3 = drive.trajectorySequenceBuilder(new Pose2d(39.04, -63.21, Math.toRadians(90.00)))
                .addDisplacementMarker(() ->{
                    RightServo.setPosition(.35);
                    LeftServo.setPosition(.65);

                })

                .lineToLinearHeading(new Pose2d(34, -58.96, Math.toRadians(90.00))) // center on first square
                .lineToLinearHeading(new Pose2d(34.79, -10.77, Math.toRadians(90))) //goes forward to 3 square
                .addTemporalMarker(.1, () -> {
                    // This marker runs two seconds into the trajectory
                    LiftMotor.setPower(.2); // was -2


                    // Run your action in here!
                })

                .lineToLinearHeading(new Pose2d(31.5, -6.5, Math.toRadians(138.24))) // turns torwards junction and move forward
                .waitSeconds(1)
                .addTemporalMarker(3, () -> {
                    // This marker runs two seconds into the trajectory
                    LiftMotor.setPower(0);


                    // Run your action in here!
                })
                //.waitSeconds(2)  cam commit
                .addTemporalMarker(2.5, () -> {

                    LiftMotor.setPower(-.8);


                })


                .addTemporalMarker(3, () -> {

                    RightServo.setPosition(.55);
                    LeftServo.setPosition(.48);
                    LiftMotor.setPower(0);

                })
                // .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(34.64, -11.50, Math.toRadians(90))) //\.a
                .lineToLinearHeading(new Pose2d(52.5,-12.5, Math.toRadians(90)))
                // .addSpatialMarker((34.64,-50))
                // .lineToLinearHeading(new Pose2d(12.67, -12.82, Math.toRadians(90.00)))



                .build();


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


        if(tagOfInterest == null){

            drive.followTrajectorySequence(park3);

        }else{
            switch(tagOfInterest.id){

                case 1:

                    drive.followTrajectorySequence(park1);
                    break;
                case 2:

                    drive.followTrajectorySequence(park2);
                    break;
                case 3:

                    drive.followTrajectorySequence(park3);
                    break;
            }
        }

    }


    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));

    }
}