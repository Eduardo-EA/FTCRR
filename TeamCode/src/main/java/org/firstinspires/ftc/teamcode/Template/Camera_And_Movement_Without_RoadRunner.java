package org.firstinspires.ftc.teamcode.Template;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoForCOmp.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Config
@Autonomous
public class Camera_And_Movement_Without_RoadRunner extends LinearOpMode
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
    // These are the names for the Motors in the code for the drive Train motor
    DcMotor RightFront;
    DcMotor LeftFront;

    DcMotor RightBack;
    DcMotor LeftBack;

    @Override
    public void runOpMode()
    {
                //These are the config for the motor class

        RightFront = hardwareMap.get(DcMotor.class,"RightFront"); //Between the Quotation mark you liable the motor for what you will configure in the Driver Hub so for this motor you will name it "RightFront" in the Driver Hub
        LeftFront = hardwareMap.get(DcMotor.class,"LeftFront");

        RightBack = hardwareMap.get(DcMotor.class,"RightBack");
        LeftBack = hardwareMap.get(DcMotor.class,"LeftBack");

        // Reverses the Left Motor copy and paste line 62 or 63 if you want to reverse any other motor
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);




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

// Will do this action when the robot hasn't detected the signal cone/April tag. Once the user press play on the Driver Hub.
        if(tagOfInterest == null){
            //Sets the Motor Power to 1 for 500 millisecond and set motor power to 0 after the 500 millisecond

          RightFront.setPower(1);
          LeftFront.setPower(1);
          RightBack.setPower(1);
          LeftBack.setPower(1);

          sleep(500);

          RightFront.setPower(0);
          LeftFront.setPower(0);
          RightBack.setPower(0);
          LeftBack.setPower(0);



        }else{
            switch(tagOfInterest.id){
            // When the robot Detects Tag 1 the robot will turn Left
                case 1:
                    RightFront.setPower(1);
                    LeftFront.setPower(-1);
                    RightBack.setPower(1);
                    LeftBack.setPower(-1);

                    sleep(500);

                    RightFront.setPower(0);
                    LeftFront.setPower(0);
                    RightBack.setPower(0);
                    LeftBack.setPower(0);
                    break;

                case 2:   // When robot detects Tag 2 the robot will turn Right
                    RightFront.setPower(-1);
                    LeftFront.setPower(1);
                    RightBack.setPower(-1);
                    LeftBack.setPower(1);

                    sleep(500);

                    RightFront.setPower(0);
                    LeftFront.setPower(0);
                    RightBack.setPower(0);
                    LeftBack.setPower(0);
                    break;

                case 3: // when the robot detects Tag 3 the robot will reverse
                    RightFront.setPower(-1);
                    LeftFront.setPower(-1);
                    RightBack.setPower(-1);
                    LeftBack.setPower(-1);

                    sleep(500);

                    RightFront.setPower(0);
                    LeftFront.setPower(0);
                    RightBack.setPower(0);
                    LeftBack.setPower(0);
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