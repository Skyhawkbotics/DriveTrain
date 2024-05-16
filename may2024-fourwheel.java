/*
Packages and Imports used for the code.
*/
package org.firstinspires.ftc;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Help;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

import java.lang.Math;



@TeleOp(name = "FourWheelTest")
public class FourWheelTest extends LinearOpMode {
  //Clock Variable
  private ElapsedTime     runtime = new ElapsedTime();

  // IMU - Includes Gyroscope / Acceleromotor / Thermometer and a lot lot more random stuff
  private BNO055IMU imu;


  //Create Motor Variables
  private DcMotorEx whl_1;
  private DcMotorEx whl_2;
  private DcMotorEx whl_3;
  private DcMotorEx whl_4;
  //private DcMotorEx arm_Rotater;
  //private CRServo servo_Claw;
  //double arm_Rotater_power = 0.0;
  //double servo_Claw_power = 0.0;
  //boolean servo_CLAW_closed = false;
  boolean left_bumper_down = false;
  boolean right_bumper_down = false;
  double robot_desiredAngle = 0.0;
  double drive_Angle = 0.0;

  Orientation orientation = null;
  Acceleration acceleration = null;
  // Max ranges from -1 to 1
  double whl_1_percent;
  double whl_2_percent;
  double whl_3_percent;
  double whl_4_percent;

  final double WHEEL_METER_CONSTANT = 0;
  final double WHEEL_INCH_CONSTANT = 0;  
  double a = 1.1;

  double last_time = runtime.seconds(); //Used to find how much time has elapsed per iteration in the runtime loop.
  double reset_last_time = runtime.seconds(); //Last time the robot has reset (N/A feature)
  double code_start_time = 0.0;
  double drive_last_time = 0.0;
  double drive_end_time = 0.0;
  boolean drive_down = false;
  double now_time = 0.0;
  private String wheelMode = "position";
  
  double clock_timer_MAX = 900000.0;
  double clock_timer = clock_timer_MAX;
  boolean clock_active = false;
  
  
  @Override
  public void runOpMode() {
    //Initalize Motors and Servos
    whl_1 = hardwareMap.get(DcMotorEx.class, "whl-1");
    whl_2 = hardwareMap.get(DcMotorEx.class, "whl-2");
    whl_3 = hardwareMap.get(DcMotorEx.class, "whl-3");
    whl_4 = hardwareMap.get(DcMotorEx.class, "whl-4");

    /*
    *arm_Rotater = hardwareMap.get(DcMotorEx.class, "armRotater");
    arm_Rotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_Rotater.setTargetPosition(0);
    arm_Rotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm_Rotater.setVelocity(1000);
    servo_Claw = hardwareMap.get(CRServo.class, "claw");
*/
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.mode                = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.accelPowerMode      = BNO055IMU.AccelPowerMode.NORMAL;
    parameters.loggingEnabled      = false;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);
    acceleration = new Acceleration();
    orientation = imu.getAngularOrientation();
    //startRobotAngle = orientation.firstAngle;
    
    telemetry.addData("Mode", "calibrating...");
    telemetry.update();
    setWheelMode("power");
    

    //AprilTag
    //initAprilTag();
    telemetry.update();


    waitForStart();
    
     if (opModeIsActive()) {
      code_start_time = runtime.seconds();
      while (opModeIsActive()) {
        double now_time = runtime.seconds();
        gamepadInputHandling(now_time);

        orientation = imu.getAngularOrientation();
        
        ////----VARIABLE MONITORING----////
        
        //telemetry.addData("armRotaterPosition", arm_Rotater_power);
        //telemetry.update();
      
        tankDriveHandling(now_time);
        //rotate()
        last_time = now_time; //To find time differentials between loops.
        //whl_corrections(); // Corrects/Adjusts power for correct results
    
        setPower();
        }
      }
    }
  
  public void setPower() {
    if (wheelMode == "power") {
      whl_1.setPower(whl_1_percent);
      whl_2.setPower(whl_2_percent);
      whl_3.setPower(-whl_3_percent);
      whl_4.setPower(whl_4_percent);
      whl_1_percent = 0;
      whl_2_percent = 0;
      whl_3_percent = 0;
      whl_4_percent = 0;
    }
    else if (wheelMode == "position") {
        whl_1.setTargetPosition((int) whl_1_percent);
        whl_2.setTargetPosition((int) whl_2_percent);
        whl_3.setTargetPosition((int) -whl_3_percent);
        whl_4.setTargetPosition((int) whl_4_percent);
    }
    telemetry.update();
  }
  
  public void gamepadInputHandling(double now_time) {
    
    if (gamepad1.left_bumper && !left_bumper_down) {
      left_bumper_down = true;
      a-=0.1;
    }
    
    else if (!gamepad1.left_bumper) {
      left_bumper_down = false;
    }
    
    if (gamepad1.right_bumper && !right_bumper_down) {
      right_bumper_down = true;
      a+=0.1;
    }
    
    else if (!gamepad1.right_bumper) {
      right_bumper_down = false;
    }
  }
  
  
  
  public void setWheelMode(String mode){
    if (mode == "position") {
      wheelMode = "position";
      whl_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      whl_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      whl_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);      
      whl_4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


      whl_1.setTargetPosition(0);
      whl_2.setTargetPosition(0);
      whl_3.setTargetPosition(0);
      whl_4.setTargetPosition(0);


      whl_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      whl_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      whl_3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      whl_4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      whl_1.setVelocity(800);
      whl_2.setVelocity(800);
      whl_3.setVelocity(800);
      whl_4.setVelocity(800);
    }
    else if (mode == "power") {
      wheelMode = "power";
      whl_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      whl_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      whl_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      whl_4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
  }
  
  public void rotateDriveHandling(double now_time, double input) {
    double multiplier = input;
    whl_1_percent += 1 * multiplier;
    whl_2_percent += 1 * multiplier;
    whl_3_percent += 1 * multiplier;
    whl_4_percent += 1 * multiplier;
  }

  public void tankDriveHandling(double now_time) {
    
    //Get Predominant Stick Position
    float l_y = gamepad1.left_stick_y;
    float l_x = gamepad1.left_stick_x;
    int planeIsY = (Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) ? 1 : 0;
    int sign = 1;
    if (planeIsY == 1 && Math.abs(gamepad1.left_stick_y) > gamepad1.left_stick_y) {
        sign = -1;
    }
    else if (planeIsY != 1 && Math.abs(gamepad1.left_stick_x) > gamepad1.left_stick_x) {
        sign = -1;
    }
    telemetry.addData("plane", planeIsY);
    telemetry.addData("sign", sign);

    if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) {
      drive_last_time = (drive_down == false) ? runtime.seconds() : drive_last_time;
      double multiplier = (gamepad1.left_stick_y);
      drive_Angle = (drive_down == false) ? imu.getAngularOrientation().firstAngle : drive_Angle;
      drive_down = true;
      if (planeIsY == 1) {
        double adjustment = Math.pow(Help.trueAngleDif(drive_Angle, imu.getAngularOrientation().firstAngle) / 40, 2);
        adjustment = (adjustment > 0.6) ? 0.6 : adjustment;
        adjustment = (adjustment < -0.6) ? -0.6 : adjustment;
        double l = 1.0 + (adjustment);
        double r = 1.0 - (adjustment);
        //l = 1.0;
      // r = 1.0;
        telemetry.addData("adjustment", adjustment);
        telemetry.addData("l", l);
        telemetry.addData("r", r);
        whl_1_percent += 1 * l_y * 0.4 * l;
        whl_2_percent += 1 * l_y * 0.4 * l;
        whl_3_percent += -1 * l_y * 0.4 * r;
        whl_4_percent += -1 * l_y * 0.4 * r;

        /*
        double mult = 100*l_y*2*(now_time-last_time);
        whl_1_percent += mult;
        whl_2_percent += mult;
        whl_3_percent -= mult;
        whl_4_percent -= mult;
        */
      }
      else if (planeIsY != 1) {
        /*
        whl_1_percent += -1  * l_x * 0.5;
        whl_2_percent += -1  * l_x * 0.5;
        whl_3_percent += 1* l_x * 0.5;
        whl_4_percent += 1 * l_x * 0.5;
        */
      }
    }
    else if (Math.abs(gamepad1.right_stick_y) > 0.2) {
        double multiplier = gamepad1.right_stick_y;
        whl_1_percent += 0.3 * multiplier;
        whl_2_percent += 0.3 * multiplier;
        whl_3_percent += 0.3 * multiplier;
        whl_4_percent += 0.3 * multiplier;
    }
    else {
      if (drive_down) {
        drive_end_time = runtime.seconds();
        drive_down = false;
      }
    }
    
    
  }

  public void rotate(String type, double angle) {
    //set whl mode to power
    //setWheelMode("power");
    
    //Based on angle difference, rotate left / right
    double angleDif = Help.trueAngleDif(angle, imu.getAngularOrientation().firstAngle);
    
    angleDif = Help.trueAngleDif(angle, imu.getAngularOrientation().firstAngle);
    //telemetry.addData("a", angleDif);
    //telemetry.addData("b", imu.getAngularOrientation().firstAngle);
    //telemetry.update();
    //Rotate to the LEFT?
    //double power = Math.clamp(Math.abs(angleDif / 45), 0.3, 1.0) * Math.signum(angleDif);
    //rotateDriveHandling(0.0, power);
    //Goal is reached, function end
    
  }
}

  