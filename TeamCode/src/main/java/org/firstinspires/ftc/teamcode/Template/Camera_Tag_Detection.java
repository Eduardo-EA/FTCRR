package org.firstinspires.ftc.teamcode.Template;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Config
@Autonomous
public class Camera_Tag_Detection extends LinearOpMode
{
    //This config the Camera
  OpenCvCamera camera;
  org.firstinspires.ftc.teamcode.AutoForCOmp.AprilTagDetectionPipeline aprilTagDetectionPipeline;

  double fx = 578.272;
  double fy = 578.272;
  double cx = 402.145;
  double cy = 221.506;

  // UNITS ARE METERS
  double tagsize = 0.166;

  // Tag ID 1,2,3 from the 36h11 family
    // If you want to change the Tags go to the PDF of the April Tags and replace the Numbers based on the one the Tags shows in the PDF

    //LEFT is for left zone, Middle is for the middle zone, and Right is right zone
  int LEFT = 1;
  int MIDDLE = 2;
  int RIGHT = 3;

  AprilTagDetection tagOfInterest = null;

  // Add Motor, servo, ect.  Config here
    //ex. Dcmotor RightMotor;


  @Override
  public void runOpMode()
  {
      SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
      // Add Motor hardware config here
      // Ex.   RightMotor = hardwareMap.get(DcMotor.class, "RightMotor");




      // Starts the camera during Initialization and scans the signal sleeve
      if (isStopRequested()) return;



      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
      aprilTagDetectionPipeline = new org.firstinspires.ftc.teamcode.AutoForCOmp.AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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
                    //Tells the user if the Tags are detected
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








      //Here you will tell the robot what to do once the tag is detected
      if(tagOfInterest == null){
        // Add Action here when the Camera does not detect the Tag

      }else{
          switch(tagOfInterest.id){

              case 1:
              //Add Action here when Camera detects Tag1
                  break;
              case 2:
                  //Add Action here when Camera detects Tag2
                  break;
              case 3:
                  //Add Action here when Camera detects Tag3
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