package org.firstinspires.ftc.teamcode.Template;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoForCOmp.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
@Autonomous
public class Camera_And_Movement_With_RoadRunner_Without_Gyro_Sensor extends LinearOpMode
{

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




    @Override
    public void runOpMode()
    {
        // TODO: Configure SampleMecanum Drive File First or this Program will not work.
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


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
        Trajectory Forward = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();

        Trajectory Backward = drive.trajectoryBuilder(new Pose2d())
                .forward(-5)
                .build();

        Trajectory StrafeRight = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(5)
                .build();

        Trajectory StrafeLeft = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(5)
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

            drive.followTrajectory(Forward);

        }else{
            switch(tagOfInterest.id){

                case 1:
                    drive.followTrajectory(StrafeLeft);
                    break;
                case 2:
                    drive.followTrajectory(StrafeRight);
                    break;
                case 3:
                    drive.followTrajectory(Backward);
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