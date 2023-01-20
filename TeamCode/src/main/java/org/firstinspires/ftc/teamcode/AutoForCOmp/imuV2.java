package org.firstinspires.ftc.teamcode.AutoForCOmp;


import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name="TEst", group="Linear Opmode")
public class imuV2 extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();  //delete final if prob
    DcMotor RightFrontDrive = null;
    DcMotor LeftFrontDrive = null;
    DcMotor RightBackDrive = null;
    DcMotor LeftBackDrive = null;


    DcMotor LiftMotor = null;

    Servo RightServo;
    Servo LeftServo;

    BNO055IMU BNO055;
    Orientation angles;

    boolean oldServoButton;
    boolean oldcontrol;


    static final double     Lift_Speed              = 1;




    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        BNO055 = hardwareMap.get(BNO055IMU.class, "IMU");
        BNO055.initialize(parameters);

        RightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        LeftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        RightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        LeftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");

        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");

        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setAutoClear(false);
        Telemetry.Item LiftMotorPosition = telemetry.addData("Lift Motor Position", LiftMotor.getCurrentPosition());

        while(!isStarted()){
            LiftMotorPosition.setValue(LiftMotor.getCurrentPosition());
            telemetry.update();
        }
        int Lifttarget = 0;
        double Liftspeed = 1;
        String LiftcurrentDirection = "Ground ";


        RightServo = hardwareMap.get(Servo.class, "RightServo");
        LeftServo = hardwareMap.get(Servo.class, "LeftServo");

        RightServo.setPosition(.35);
        LeftServo.setPosition(.65);

        oldServoButton = false;
        oldcontrol = false;


        RightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        RightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a) {

                // Get the calibration data
                BNO055IMU.CalibrationData calibrationData = BNO055.readCalibrationData();

                // Save the calibration data to a file. You can choose whatever file
                // name you wish here, but you'll want to indicate the same file name
                // when you initialize the IMU in an opmode in which it is used. If you
                // have more than one IMU on your robot, you'll of course want to use
                // different configuration file names for each.
                String filename = "AdafruitIMUCalibration.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                telemetry.log().add("saved to '%s'", filename);

                // Wait for the button to be released
                while (gamepad1.a) {
                    telemetry.update();
                    idle();
                }
            }

            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            angles = BNO055.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float pi = (float) 3.14159;
            float gyroRadians = angles.firstAngle * pi / 180;

            float axial2 = (float) (axial * cos(gyroRadians) - lateral * sin(gyroRadians));
            lateral = axial * sin(gyroRadians) + lateral * cos(gyroRadians);
            axial = axial2;

            double leftFrontSign  = Math.signum((axial + lateral + yaw));
            double rightFrontSign = Math.signum((axial - lateral - yaw));
            double leftBackSign   = Math.signum((axial - lateral + yaw));
            double rightBackSign  = Math.signum((axial + lateral - yaw));

            double leftFrontPower  = /*leftFrontSign */ pow((axial + lateral + yaw), 3);
            double rightFrontPower = /*rightFrontSign */ pow((axial - lateral - yaw), 3);
            double leftBackPower   = /*leftBackSign */ pow((axial - lateral + yaw), 3);
            double rightBackPower  = /*rightBackSign */ pow((axial + lateral - yaw), 3);


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            LeftFrontDrive.setPower(leftFrontPower *2/3);
            RightFrontDrive.setPower(rightFrontPower *2/3);
            LeftBackDrive.setPower(leftBackPower *2/3);
            RightBackDrive.setPower(rightBackPower *2/3);



            //grab cone
            if (gamepad2.right_bumper) {
                RightServo.setPosition(.35);
                LeftServo.setPosition(.65);
                //let go of cone
            } else if (gamepad2.left_bumper){
                RightServo.setPosition(.55);
                LeftServo.setPosition(.48);
            }
            

            LiftMotor.setPower(gamepad2.right_trigger);
            LiftMotor.setPower(-gamepad2.left_trigger);

            if (gamepad2.dpad_up) { //Lift up High
                Lifttarget = 200;
                Liftspeed = 1;
                LiftcurrentDirection = "High";
                LiftMotor.setPower(Liftspeed);
                LiftMotor.setTargetPosition(Lifttarget);
            } else if (gamepad2.dpad_right) {   // mid
              Lifttarget = 100;
              Liftspeed = 1;
              LiftcurrentDirection = "mid";
                LiftMotor.setPower(Liftspeed);
                LiftMotor.setTargetPosition(Lifttarget);
            } else if (gamepad2.dpad_down){ //low
                Lifttarget = 50;
                Liftspeed = 1;
                LiftcurrentDirection = "mid";
                LiftMotor.setPower(Liftspeed);
                LiftMotor.setTargetPosition(Lifttarget);
            }
            if ( LiftcurrentDirection == "down" && (LiftMotor.getTargetPosition() < 5)) {
                Liftspeed = 0;
                LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            idle();

            if (LiftMotor.isBusy()) {
                LiftMotorPosition.setValue(LiftMotor.getCurrentPosition());
                telemetry.update();
            }

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("YAW", angles.firstAngle);
            telemetry.addData("Starting at",  "%7d :%7d", LiftMotor.getCurrentPosition());
            telemetry.update();
            telemetry.update();


        }
    }

}