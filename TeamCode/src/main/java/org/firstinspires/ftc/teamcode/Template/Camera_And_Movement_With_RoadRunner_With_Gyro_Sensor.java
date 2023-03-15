package org.firstinspires.ftc.teamcode.Template;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoForCOmp.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

// TODO: To Have the robot to be Accurate you will need to Tune the PID. [To learn how to tune the PID look at the RoadRunner Document--- https://learnroadrunner.com]
@Config
@Autonomous
public class Camera_And_Movement_With_RoadRunner_With_Gyro_Sensor extends LinearOpMode
{

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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




    @Override
    public void runOpMode()
    {
        // TODO: Configure SampleMecanum Drive File First or this Program will not work.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Tells the robot were the robot starting position is
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

        // Robot will move forward with the gyro sensor to keep robot straight
        TrajectorySequence Forward = drive.trajectorySequenceBuilder(new Pose2d(39.04, -63.21, Math.toRadians(90.00)))

                .lineToLinearHeading(new Pose2d(39, -40, Math.toRadians(90.00))) // center on first square

                .build();
        TrajectorySequence StrafeLeft = drive.trajectorySequenceBuilder(new Pose2d(39.04, -63.21, Math.toRadians(90.00)))

                .lineToLinearHeading(new Pose2d(20, -63, Math.toRadians(90.00))) // center on first square

                .build();
        TrajectorySequence StrafeRight = drive.trajectorySequenceBuilder(new Pose2d(39.04, -63.21, Math.toRadians(90.00)))

                .lineToLinearHeading(new Pose2d(50, -63, Math.toRadians(90.00))) // center on first square

                .build();
        TrajectorySequence Backward = drive.trajectorySequenceBuilder(new Pose2d(39.04, -63.21, Math.toRadians(90.00)))

                .lineToLinearHeading(new Pose2d(39, -70, Math.toRadians(90.00))) // center on first square

                .build();



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

            drive.followTrajectorySequence(Forward);

        }else{
            switch(tagOfInterest.id){

                case 1:
                    drive.followTrajectorySequence(StrafeLeft);
                    break;
                case 2:
                    drive.followTrajectorySequence(StrafeRight);
                    break;
                case 3:
                    drive.followTrajectorySequence(Backward);
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