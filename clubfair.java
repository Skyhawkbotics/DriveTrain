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



@TeleOp(name = "2024 TEST")
public class mechanumdrive extends LinearOpMode {
  //Clock Variable
  private ElapsedTime     runtime = new ElapsedTime();

  // IMU - Includes Gyroscope / Acceleromotor / Thermometer and a lot lot more random stuff
  private BNO055IMU imu;


  //Create Motor Variables
  private DcMotorEx whl_LB;
  private DcMotorEx whl_LF;
  private DcMotorEx whl_RB;
  private DcMotorEx whl_RF;
  private DcMotorEx candy_SHOOTER;
  private CRServo servo_ROTATER;
  private CRServo servo_DRONE;
  private CRServo servo_DRONE2;
  double servo_ROTATER_power = 0.0;

  double arm_HOOKDOWN_speed = 0.0;
  double arm_HOOKUP_speed =0.0;
  private CRServo servo_CLAW;
  double servo_CLAW_power = 0.0;
  boolean servo_CLAW_closed = false;
  boolean right_bumper_DOWN = false;
  boolean y_down = false;
  double y_last_held = -1.0;

  private DcMotorEx claw_ELEVATOR1;
  private DcMotorEx claw_ELEVATOR2;
  double claw_ELEVATOR_position = 0.0;
  double candy_SHOOTER_position = 0.0;

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
  
  @Override
  public void runOpMode() {
    //Initalize Motors and Servos
    whl_LB = hardwareMap.get(DcMotorEx.class, "whl_1");
    whl_LF = hardwareMap.get(DcMotorEx.class, "whl_4");
    whl_RB = hardwareMap.get(DcMotorEx.class, "whl_2");
    whl_RF = hardwareMap.get(DcMotorEx.class, "whl_3");

    candy_SHOOTER = hardwareMap.get(DcMotorEx.class, "candy");
    
    candy_SHOOTER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    candy_SHOOTER.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    candy_SHOOTER_position = candy_SHOOTER.getCurrentPosition();
    //candy_SHOOTER.setVelocity(2000);

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
    
    // Wait for the DS start button to be touched.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.update();


    waitForStart();
    
     if (opModeIsActive()) {
      code_start_time = runtime.seconds();
      while (opModeIsActive()) {
        
        //AprilTag Stuff START
        int aprilTagsDetected = 0;
        double[][] newAprilTagInfos = null;
        if (iterations % 1 == 0 ) {
        //  newAprilTagInfos = telemetryAprilTag();
          if (newAprilTagInfos != null) {
            aprilTagInfos = newAprilTagInfos;
          }
        }

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

        if (y_last_held != -1.0) {
          double delta = runtime.seconds()-y_last_held;
          if (delta < 0.1) {
            servo_DRONE_power = -0.5;
          }
          else if (delta > 0.1 && delta < 0.4) {
            servo_DRONE_power = 0;
          }
          else if (delta > 0.4) {
            servo_DRONE_power = 1;
          }
          if (delta > 0.8) {
            servo_DRONE_power = 0;
            y_last_held = -1.0;
          }

        }
        
        //now_time, the time since the start of the program and is used to find time differentials between loop iterations
        if (clock_timer >= 0.0) {
          gamepadInputHandling(now_time);
        }
        clock(now_time);
        last_time = now_time; //To find time differentials between loops.
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
        telemetry.addData("candy_SHOOTER_currentPosition", candy_SHOOTER.getCurrentPosition());
        telemetry.addData("candy_SHOOTER_position", candy_SHOOTER_position);
        telemetry.update();
    
        tankDriveHandling();
    
        whl_corrections(); // Corrects/Adjusts power for correct results
        
        //Set power of motors to their corresponding variables when clock is 0
        if (clock_timer <= 0) {
          whl_LB_percent = 0;
          whl_RB_percent = 0;
          whl_LF_percent = 0;
          whl_RF_percent = 0;
        }
        setPower();
        }
      }
    // Save more CPU resources when camera is no longer needed.
    visionPortal.close();
    }
  
  public void setPower() {
    if (wheelMode == "power") {
      whl_LB.setPower(whl_LB_percent);
      whl_RB.setPower(-whl_RB_percent);
      whl_LF.setPower(-whl_LF_percent);
      whl_RF.setPower(whl_RF_percent);
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
    //candy_SHOOTER.setTargetPosition((int)candy_SHOOTER_position);
    //telemetry.update();
  }
  
  public void gamepadInputHandling(double now_time) {
    if (gamepad1.x) {
      start_down = true;
    }
    else {
      if (start_down) {
        alignRobot = !alignRobot;
        aprilTagInfos = null;
      }
      start_down = false;
    }
    //Rotate 90
    // If there are any joystick moveemnts during this, cancel rotation
    if (gamepad1.dpad_right && !rightangle_active) {
      rightangle_active = true;
      desiredRobotAngle = Help.angleCorrection(imu.getAngularOrientation().firstAngle, -90);

    }
    else if (gamepad1.dpad_left && !rightangle_active){
      rightangle_active = true;
      desiredRobotAngle = Help.angleCorrection(imu.getAngularOrientation().firstAngle, 90);
    }
    
    if (gamepad1.y) {
      if (candy_SHOOTER.getCurrentPosition() <= candy_SHOOTER_position+20.0) {
        candy_SHOOTER.setVelocity(10000);
      }
      else if (candy_SHOOTER.getCurrentPosition() > candy_SHOOTER_position+50.0) {
        candy_SHOOTER.setVelocity(-200);
      }
      else {
        candy_SHOOTER.setVelocity(0);
      }
    }
    else {
      if (candy_SHOOTER.getCurrentPosition() >= candy_SHOOTER_position+ 4.0) {
        candy_SHOOTER.setVelocity(-500);
        }
      else {
        candy_SHOOTER.setVelocity(0);
      }
    }
    
    
    if (!gamepad2.left_bumper) {
      left_bumper_DOWN = false;
    }

    if (gamepad2.right_bumper && claw_ELEVATOR_position <470) {
      if (!gamepad1.b) {
         claw_ELEVATOR_position+= 150 * (now_time-last_time);
      }
      else{
        claw_ELEVATOR_position = 470;
      }
    }
    else if (gamepad2.right_trigger > 0.2 && claw_ELEVATOR_position >-35) {
      if (!gamepad1.b) {
        claw_ELEVATOR_position-= 150 * (now_time-last_time);
      }
      else {
        claw_ELEVATOR_position=-35;
      }
    }
  }
  
  public void clock(double now_time) {
    if (gamepad2.start) {
      clock_active = false;
      clock_timer = clock_timer_MAX;
    }
    else if (!clock_active && !gamepad1.atRest()) {
      clock_active = true;
    }
    
    if (clock_active) {
      clock_timer -= (now_time-last_time);
      if (clock_timer < 0.0)
        clock_timer = 0.0;
    }
  }
  
  public void whl_corrections() {
    //1st mult: individual wheel balance
    //2nd mult: better rotation (weaker front wheels)
    //3rd mult: weaker overall wheels
      whl_RF_percent = (float) (whl_RF_percent * 0.8 * 1 * 0.6);
      whl_RB_percent = (float) (whl_RB_percent * 0.8 *1* 0.6);
      whl_LF_percent = (float) (whl_LF_percent * 0.8 *1 * 0.6);
      whl_LB_percent = (float) (whl_LB_percent * 0.8 *1*0.6);
      
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
  
  //Ready
  
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

    if (gamepad1.left_trigger > 0.8) {
      whl_LB_percent += 1;
      whl_LF_percent -= 0.9;
      whl_RB_percent -= 1;
      whl_RF_percent += 0.9;
      if (!isStrafing){
        isStrafing = true;
        strafeStartingAngle = imu.getAngularOrientation().firstAngle;
      }
    }
    else if (gamepad1.right_trigger > 0.8) {
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
   
   private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private double[][] telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        //telemetry.addData("# AprilTags Detected", currentDetections.size());
        double[][] aprilTagInfos = new double[10][9];
        // Step through the list of detections and display info for each one.
        int iteration = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                aprilTagInfos[iteration][0] = detection.ftcPose.x;
                aprilTagInfos[iteration][1] = detection.ftcPose.y;
                aprilTagInfos[iteration][2] = detection.ftcPose.z;
                aprilTagInfos[iteration][3] = detection.ftcPose.pitch;
                aprilTagInfos[iteration][4] = detection.ftcPose.roll;
                aprilTagInfos[iteration][5] = detection.ftcPose.yaw;
                aprilTagInfos[iteration][6] = detection.ftcPose.range;
                aprilTagInfos[iteration][7] = detection.ftcPose.bearing;
                aprilTagInfos[iteration][8] = detection.ftcPose.elevation;
                iteration+=1;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                
                telemetry.update();
            }
        }   // end for() loop

        // Add "key" information to telemetry
        /*
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
*/
        //telemetry.update();
        aprilTagInfos[9][0] = (double) iteration;
        return aprilTagInfos;
    }   // end method telemetryAprilTag()
   
   
  }
  
