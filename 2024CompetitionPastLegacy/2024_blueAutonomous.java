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
public class Blue2024 extends LinearOpMode {
  //Clock Variable
  private ElapsedTime     runtime = new ElapsedTime();

  // IMU - Includes Gyroscope / Acceleromotor / Thermometer and a lot lot more random stuff
  private BNO055IMU imu;

  private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/BlueAiModel.tflite";
    private static final String[] LABELS = {
        "BlueLineRightWithObject",
        "BlueLineCenterWithObject",
        "BlueLineLeftWithObject"
    };
  int LeftObjectDetected = 0;
  int CenterObjectDetected = 0;
  int RightObjectDetected = 0;

  private TfodProcessor tfod;

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
  int cameraCaptures = 0;

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

    initTfod();

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

    //AprilTag
    //initAprilTag();
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
        
        // Start instructions
        //Open tfod feed for 5 seconds & drive forward like a bit
        if (runtime.seconds() - code_start_time < 5) {
            if (!autoInitAction) {
                autoInitAction = true;
                setWheelMode("position");
                //autoDriveHandling(-0.15,-0.15,-0.15,-0.15); //drive forward 15cm
            }
            if ((runtime.seconds() - code_start_time) - (cameraCaptures/4) > 0){
              telemetryTfod();
              cameraCaptures+=1;
            }
        }
        else {
          if (!cameraClosed) {
            cameraClosed=true;
            visionPortal.close();
          }
        }
        /*
        else {
          if (!cameraClosed) {
            cameraClosed = true;
           //visionPortal.stopStreaming();
           //visionPortal.close();
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
        telemetry.addData("left", LeftObjectDetected);
        telemetry.addData("cent", CenterObjectDetected);
        telemetry.addData("right", RightObjectDetected);

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
  
      whl_LB.setVelocity(500);
      whl_RB.setVelocity(500);
      whl_LF.setVelocity(500);
      whl_RF.setVelocity(500);
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
   
   private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            // With the following lines commented out, the default TfodProcessor Builder
            // will load the default model for the season. To define a custom model to load, 
            // choose one of the following:
            //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
            //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            .setModelFileName(TFOD_MODEL_FILE)

            // The following default settings are available to un-comment and edit as needed to 
            // set parameters for custom models.
            .setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.5f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
    //Variables needed to be saved: label
   private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            if (recognition.getLabel() == "BlueLineRightWithObject") {
                RightObjectDetected +=1;
            }
            else if (recognition.getLabel() == "BlueLineCenterWithObject") {
                CenterObjectDetected +=1;
            }
            else if (recognition.getLabel() == "BlueLineLeftWithObject") {
                LeftObjectDetected +=1;
            }
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
  }
  