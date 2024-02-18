/*
Packages and Imports used for the code.
*/
package org.firstinspires.ftc;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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



@Autonomous(name = "2024 Blue Prop Code")
public class RED_AUTO extends LinearOpMode {
  //Clock Variable
  private ElapsedTime     runtime = new ElapsedTime();

  // IMU - Includes Gyroscope / Acceleromotor / Thermometer and a lot lot more random stuff
  private BNO055IMU imu;


  //Create Motor Variables
  private DcMotorEx whl_LB;
  private DcMotorEx whl_LF;
  private DcMotorEx whl_RB;
  private DcMotorEx whl_RF;
  //private DcMotorEx arm_ELEVATOR;
  private DcMotorEx arm_HOOKUP;
  private DcMotorEx arm_HOOKDOWN;
  private CRServo servo_ROTATER;
  private CRServo servo_DRONE;
  double servo_ROTATER_power = 0.0;
 // private CRServo servo_DRONE2;
 // double servo_DRONE2_power = 0.0;

  private CRServo servo_CLAW;
  double servo_CLAW_power = 0.0;
  boolean servo_CLAW_closed = false;
  boolean right_bumper_DOWN = false;

  private DcMotorEx claw_ELEVATOR1;
  private DcMotorEx claw_ELEVATOR2;
  double claw_ELEVATOR_position = 0.0;
  double servo_DRONE_power = 0.0;

  // Max ranges from -1 to 1
  double whl_LB_percent;
  double whl_LF_percent;
  double whl_RB_percent;
  double whl_RF_percent;

  final double WHEEL_METER_CONSTANT = 578.97;
  final double WHEEL_INCH_CONSTANT = (1 / 34) * 500;  


  double last_time = runtime.seconds(); //Used to find how much time has elapsed per iteration in the runtime loop.
  double reset_last_time = runtime.seconds(); //Last time the robot has reset
  //double arm_ELEVATOR_speed = 0.0;

  private String wheelMode = "power";
  
  double clock_timer_MAX = 900000.0;
  double clock_timer = clock_timer_MAX;
  boolean clock_active = false;
  boolean alignRobot = false;
  boolean start_down = false;
  
  boolean isStrafing = false;
  double strafeStartingAngle = -1000.0;
  double strafeEndTime =0.0;
  
  double startRobotAngle = 0.0;
  Orientation orientation = null;
  Acceleration acceleration = null;

  int iterations = 0;
  
  //Presets
  boolean joystick_active = false;
  boolean rightangle_active = false;
  double code_start_time = 0.0;
  double uncode_start_time = 0.0;
  boolean left_bumper_DOWN = false;

  boolean autoInitAction = false;
  boolean autoFirstAction = false;
  boolean a2 = false;
  boolean cameraClosed = false;

  //aprilTag setup
  private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

  /**
   * The variable to store our instance of the AprilTag processor.
   */
   private AprilTagProcessor aprilTag;
   /**
    * The variable to store our instance of the vision portal.
   */
   private VisionPortal visionPortal;
   double[][] aprilTagInfos;
   double desiredRobotAngle = 0.0;
   boolean a_left = false;
   boolean a_right = true;
  
  @Override
  public void runOpMode() {

    //initTfod();

    //Initalize Motors and Servos
    whl_LB = hardwareMap.get(DcMotorEx.class, "left/back");
    whl_LF = hardwareMap.get(DcMotorEx.class, "left/front");
    whl_RB = hardwareMap.get(DcMotorEx.class, "right/back");
    whl_RF = hardwareMap.get(DcMotorEx.class, "right/front");
    
    //arm_ELEVATOR = hardwareMap.get(DcMotorEx.class, "Arm Extender");
    
    claw_ELEVATOR1 = hardwareMap.get(DcMotorEx.class, "Left String Uppy Puller");
    claw_ELEVATOR2 = hardwareMap.get(DcMotorEx.class, "Right String Uppy Puller");

    arm_HOOKUP = hardwareMap.get(DcMotorEx.class, "Hook Arm Up");
    arm_HOOKDOWN = hardwareMap.get(DcMotorEx.class, "Hook Arm Down");
    servo_ROTATER = hardwareMap.get(CRServo.class, "Claw Flipper");
    servo_CLAW = hardwareMap.get(CRServo.class, "Claw Opener");
    //arm_ELEVATOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //arm_ELEVATOR.setTargetPosition(0);
    //arm_ELEVATOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //arm_ELEVATOR.setVelocity(10000);

    servo_DRONE = hardwareMap.get(CRServo.class, "Drone Launcher");
    //servo_DRONE2 = hardwareMap.get(CRServo.class, "Drone Launcher 2");
    
    claw_ELEVATOR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    claw_ELEVATOR1.setTargetPosition(0);
    claw_ELEVATOR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    claw_ELEVATOR1.setVelocity(700);

    claw_ELEVATOR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    claw_ELEVATOR2.setTargetPosition(0);
    claw_ELEVATOR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    claw_ELEVATOR2.setVelocity(700);
    
    arm_HOOKUP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_HOOKUP.setTargetPosition(0);
    arm_HOOKUP.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm_HOOKUP.setVelocity(700);

    arm_HOOKDOWN.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_HOOKDOWN.setTargetPosition(0);
    arm_HOOKDOWN.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm_HOOKDOWN.setVelocity(700);

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
    startRobotAngle = orientation.firstAngle;
    
    telemetry.addData("Mode", "calibrating...");
    telemetry.update();
    setWheelMode("power");
    
    //April Tag Testing
    
    //Blegh myDetector = AprilTagIdCode.createAprilTagDetector("Camera1");
    //AprilTagIdCode.startAprilTagDetector(
      
    //);

    // Wait for the DS start button to be touched.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.update();


    waitForStart();
    
     if (opModeIsActive()) {
      // Start the loop
      //rotate("", startRobotAngle-90.0);
      //autoDriveHandling(500,500,500,500);
      code_start_time = runtime.seconds();
      while (opModeIsActive()) {
        

          if (runtime.seconds() - code_start_time < 1 && !autoInitAction) {
            autoInitAction = true;
            setWheelMode("position");
            autoDriveHandling(0.2,-0.2,0.2,-0.2);
          }
          else if (runtime.seconds() - code_start_time > 1 && runtime.seconds() - code_start_time < 3.5) {
            tankDriveHandling();
          }

        // Start instructions
        //Open tfod feed for 5 seconds & drive forward like a bit
        /*
        if (runtime.seconds() - code_start_time < 5) {
            if (!autoInitAction) {
                autoInitAction = true;
                setWheelMode("position");
                autoDriveHandling(0.5,-0.5,0.5,-0.5);
            }
            //telemetryTfod();
        }
        else {
          if (!cameraClosed) {
            cameraClosed = true;
           v//isionPortal.stopStreaming();
           v//isionPortal.close();
          }
          if (LeftObjectDetected > CenterObjectDetected && LeftObjectDetected > RightObjectDetected) {
              //Set Angle
              if (!autoFirstAction) {
                  desiredRobotAngle = Help.angleCorrection(imu.getAngularOrientation().firstAngle, -45);
                  autoFirstAction = true;
                  setWheelMode("power");
              }
              if (!a2)
                  rotate("", desiredRobotAngle);
              if (!a2 && runtime.seconds() - code_start_time > 7) {
                  a2 = true;
                  setWheelMode("position");
                  autoDriveHandling(0.5,0.5,0.5,0.5);
              } 
          }
          else if (CenterObjectDetected > LeftObjectDetected && CenterObjectDetected > RightObjectDetected) {
              //Go Middle
              //Set Angle
              if (!autoFirstAction) {
                  desiredRobotAngle = Help.angleCorrection(imu.getAngularOrientation().firstAngle, 0);
                  autoFirstAction = true;
                  setWheelMode("power");
              }
              if (!a2)
                  rotate("", desiredRobotAngle);
              if (!a2 && runtime.seconds() - code_start_time > 7) {
                  a2 = true;
                  setWheelMode("position");
                  autoDriveHandling(0.5,0.5,0.5,0.5);
              } 
          }
          else {
              //Go Right
              //Set Angle
              if (!autoFirstAction) {
                  desiredRobotAngle = Help.angleCorrection(imu.getAngularOrientation().firstAngle, 45);
                  autoFirstAction = true;
                  setWheelMode("power");
              }
              if (!a2)
                  rotate("", desiredRobotAngle);
              if (!a2 && runtime.seconds() - code_start_time > 7) {
                  a2 = true;
                  setWheelMode("position");
                  autoDriveHandling(0.5,0.5,0.5,0.5);
              } 
          }
        }*/

        if (rightangle_active) {
          if (Math.abs(Help.trueAngleDif(desiredRobotAngle,imu.getAngularOrientation().firstAngle)) > 5){
            rotate("", desiredRobotAngle);
          }
          else {
            rightangle_active = false;
          }
        }
        double now_time = runtime.seconds();
        if (!isStrafing && strafeStartingAngle != -1000.0 && now_time-strafeEndTime > 0.5) {
          desiredRobotAngle = strafeStartingAngle;
          if (Math.abs(Help.trueAngleDif(desiredRobotAngle,imu.getAngularOrientation().firstAngle)) > 5){
            rotate("", desiredRobotAngle);
          }
          else {
            strafeStartingAngle = -1000.0;
          }
        }
        orientation = imu.getAngularOrientation();
        iterations +=1;

        
        ////----VARIABLE MONITORING----////
        
        telemetry.addData("orientation", orientation);
        telemetry.addData("velocity", imu.getVelocity());
        telemetry.addData("acceleration_x", imu.getAcceleration().xAccel);
        telemetry.addData("acceleration_y", imu.getAcceleration().yAccel);
        telemetry.addData("acceleration_z", imu.getAcceleration().zAccel);

        telemetry.addData("firstAngle", imu.getAngularOrientation().firstAngle);
        telemetry.addData("desiredAngle", desiredRobotAngle);
        telemetry.addData("angleDifference", Help.trueAngleDif(desiredRobotAngle,imu.getAngularOrientation().firstAngle));

        telemetry.addData("90 degree active", rightangle_active);
        telemetry.addData("strafeCorrection", strafeStartingAngle);
        telemetry.addData("strafeEndTimeDifferential", runtime.seconds() - strafeEndTime);
        telemetry.update();
        
        
        setPower();
        }
      }
    // Save more CPU resources when camera is no longer needed.
    //visionPortal.stopStreaming();
    //visionPortal.close();
    }
  
  public void setPower() {
    if (wheelMode == "power") {
      whl_LB.setPower(whl_LB_percent);
      whl_RB.setPower(whl_RB_percent);
      whl_LF.setPower(-whl_LF_percent);
      whl_RF.setPower(-whl_RF_percent);
      whl_LB_percent = 0;
      whl_RB_percent = 0;
      whl_LF_percent = 0;
      whl_RF_percent = 0;
    }
    else if (wheelMode == "position") {
        whl_LB.setTargetPosition((int) whl_LB_percent);
        whl_RB.setTargetPosition((int) whl_RB_percent);
        whl_LF.setTargetPosition((int) -whl_LF_percent);
        whl_RF.setTargetPosition((int) -whl_RF_percent);
    }
    //arm_ELEVATOR.setTargetPosition((int)arm_ELEVATOR_speed);
    claw_ELEVATOR1.setTargetPosition((int)claw_ELEVATOR_position);
    claw_ELEVATOR2.setTargetPosition((int)claw_ELEVATOR_position);
    //arm_HOOKUP.setTargetPosition((int)arm_HOOKUP_speed);
    //arm_HOOKDOWN.setTargetPosition((int)arm_HOOKDOWN_speed);
    servo_ROTATER.setPower(servo_ROTATER_power);
    servo_CLAW.setPower(servo_CLAW_power);
    servo_DRONE.setPower(servo_DRONE_power);

    //claw_GRIP.setPower(claw_GRIP_angle);
    //telemetry.update();
  }
  
  
  
  public void whl_corrections() {
    //1st mult: individual wheel balance
    //2nd mult: better rotation (weaker front wheels)
    //3rd mult: weaker overall wheels
      whl_RF_percent = (float) (whl_RF_percent * 0.8 * 1 * 0.6);
      whl_RB_percent = (float) (whl_RB_percent * 0.8 *1* 0.6);
      whl_LF_percent = (float) (whl_LF_percent * 0.8 *1 * 0.6);
      whl_LB_percent = (float) (whl_LB_percent * 0.8 *1*0.6);
      
      /* STRAFE
      whl_RF_percent = (float) (whl_RF_percent * 0.6 * 0.6);
      whl_RB_percent = (float) (whl_RB_percent * -0.5 *0.8);
      whl_LF_percent = (float) (whl_LF_percent * 0.65 *0.6);
      whl_LB_percent = (float) (whl_LB_percent * 0.6 *0.8);
      */
  }
  
  public void setWheelMode(String mode){
    if (mode == "position") {
      wheelMode = "position";
      whl_LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      whl_RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      whl_LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      whl_RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  
      whl_LB.setTargetPosition(0);
      whl_RB.setTargetPosition(0);
      whl_LF.setTargetPosition(0);
      whl_RF.setTargetPosition(0);
  
      whl_LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      whl_RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      whl_LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      whl_RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  
      whl_LB.setVelocity(300);
      whl_RB.setVelocity(300);
      whl_LF.setVelocity(300);
      whl_RF.setVelocity(300);
    }
    else if (mode == "power") {
      wheelMode = "power";
      whl_LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      whl_RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      whl_LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      whl_RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
  }
  
  public void autoDriveHandling(double LB, double LF, double RB, double RF) {
    whl_LB_percent += LB*WHEEL_METER_CONSTANT;
    whl_LF_percent += LF*WHEEL_METER_CONSTANT;
    whl_RB_percent += RB*WHEEL_METER_CONSTANT;
    whl_RF_percent += RF*WHEEL_METER_CONSTANT;
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
    if (angleDif > 0) {
      //Rotate to the LEFT?
      double power= -1 * Math.abs(angleDif / 45);
      power = (power < -1) ? -1 : power;
      power = (power > -0.7) ? -0.7 : power;
      twoDriveHandling(0, power);
    }
    else if (angleDif < 0) {
      //Rotate to the RIGHT?
      double power = 1 * Math.abs(angleDif / 45);
      power = (power >1) ? 1 : power;
      power = (power <0.7) ? 0.7 : power;
      twoDriveHandling(0, power);
    }
    //Goal is reached, function end
    
  }
  public void twoDriveHandling(double Y, double X) {
    whl_LB_percent = 0;
    whl_LF_percent = 0;
    whl_RB_percent = 0;
    whl_RF_percent = 0;
    
    whl_LB_percent += Y;
    whl_LF_percent += Y;
    whl_RB_percent += Y;
    whl_RF_percent += Y;
    
    whl_LB_percent += X;
    whl_LF_percent += X;
    whl_RB_percent -= X;
    whl_RF_percent -= X;
    
    whl_LB_percent = whl_LB_percent/1.5;
    whl_LF_percent = whl_LF_percent/1.5;
    whl_RB_percent = whl_RB_percent/1.5;
    whl_RF_percent = whl_RF_percent/1.5;
  }
  //Ready
  public void tankDriveHandling() {

    if (Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.right_stick_y) > 0.2){
      joystick_active = true;
    }
    else {
      joystick_active = false;
    }

    boolean dif = Math.abs((gamepad1.left_stick_y+gamepad1.left_stick_x))>Math.abs((gamepad1.right_stick_x+gamepad1.right_stick_y));
    
    float drv_stick_y2 = gamepad1.right_stick_y;
    float drv_stick_x2 = gamepad1.right_stick_x;
    float truth = (Math.abs(gamepad1.right_stick_y) - Math.abs(gamepad1.left_stick_y) > 0) ? gamepad1.right_stick_y : gamepad1.left_stick_y;
  
    

    /*
    if (gamepad1.dpad_right) {
      whl_RF_percent = 2;
      whl_RB_percent = -1.5f;
      whl_LF_percent = -2;
      whl_LB_percent = 1.5f;
    }
    
    else if (gamepad1.dpad_left) {
      whl_LF_percent = 2;
      whl_LB_percent = -1.5f;
      whl_RB_percent = 1.5f;
      whl_RF_percent = -2;
    }*/
    
    if (gamepad1.left_stick_y > 0.9 && gamepad1.right_stick_y < -0.9) {
      whl_RF_percent += -1;
      whl_RB_percent += -1f;
      whl_LF_percent += 1;
      whl_LB_percent += 1f;
    }
    
    else if (gamepad1.left_stick_y < -0.9 && gamepad1.right_stick_y > 0.9) {
      whl_LF_percent += -1;
      whl_LB_percent += -1f;
      whl_RB_percent += 1f;
      whl_RF_percent += 1;
    }
    else {
      whl_LB_percent += gamepad1.left_stick_y;
      whl_LF_percent += gamepad1.left_stick_y;
      whl_RB_percent += gamepad1.right_stick_y;
      whl_RF_percent += gamepad1.right_stick_y;
    }

    if (gamepad1.right_bumper) {
      whl_LB_percent += 0.5;
      whl_LF_percent += 0.5;
      whl_RB_percent += 0.5;
      whl_RF_percent += 0.5;
    }
    else if (gamepad1.left_bumper) {
      whl_LB_percent -= 0.5;
      whl_LF_percent -= 0.5;
      whl_RB_percent -= 0.5;
      whl_RF_percent -= 0.5;
    }

    if (a_left) {
      whl_LB_percent += 1;
      whl_LF_percent -= 0.9;
      whl_RB_percent -= 1;
      whl_RF_percent += 0.9;
      if (!isStrafing){
        isStrafing = true;
        strafeStartingAngle = imu.getAngularOrientation().firstAngle;
      }
    }
    else if (a_right) {
      whl_LB_percent -= 1;
      whl_LF_percent += 0.9;
      whl_RB_percent += 1;
      whl_RF_percent -= 0.9;
      if (!isStrafing){
        isStrafing = true;
        strafeStartingAngle = imu.getAngularOrientation().firstAngle;
      }
    }
    else if (isStrafing) {
      isStrafing = false;
      strafeEndTime = runtime.seconds();
    }
   }
  
   

    /**
     * Add telemetry about AprilTag detections.
     */

   
  }
  
