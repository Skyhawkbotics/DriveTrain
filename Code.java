//arm slides down, please fix!!!!!!!!!!!!
  
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "mechanumdrive (Blocks to Java)")
public class mechanumdrive extends LinearOpMode {

  private ElapsedTime     runtime = new ElapsedTime();
  private DcMotor leftback;
  private DcMotor leftfront;
  private DcMotor rightback;
  private DcMotor rightfront;
  private DcMotorEx _pseudo_arm;
  private DcMotorEx armrotater;
  private Servo claw;

  //private DistanceSensor distance;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    
    float left_back_pow;
    float left_front_pow;
    float right_back_pow;
    float right_front_pow;
    float arm_desiredangle = 0;
    float armrotate_desiredangle = 0;
    double claw_desiredangle = 0;
    double last_time = runtime.seconds();


    leftback = hardwareMap.get(DcMotor.class, "left/back");
    leftfront = hardwareMap.get(DcMotor.class, "left/front");
    rightback = hardwareMap.get(DcMotor.class, "right/back");
    rightfront = hardwareMap.get(DcMotor.class, "right/front");
    _pseudo_arm = hardwareMap.get(DcMotorEx.class, "_pseudo_arm");
    armrotater = hardwareMap.get(DcMotorEx.class, "armrotater");
    claw = hardwareMap.get(Servo.class, "claw");
    

    //distance = hardwareMap.get(DistanceSensor.class, "Distance");

    
    
    
    
    
    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    leftback.setDirection(DcMotorSimple.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
    
    _pseudo_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    _pseudo_arm.setTargetPosition(Help.degreesToTick(0));

    _pseudo_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
    _pseudo_arm.setVelocity(150);
    
    armrotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armrotater.setTargetPosition(Help.degreesToTick(0));

    armrotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    armrotater.setVelocity(1200);

    waitForStart();
    
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        double now_time = runtime.seconds();
        if (gamepad1.dpad_down) {
          arm_desiredangle-=100 * (now_time-last_time);
        } 
        if (gamepad1.dpad_up) {
          arm_desiredangle+=100 * (now_time-last_time);
        }
         if (gamepad1.a) {
          armrotate_desiredangle-=800 * (now_time-last_time);
        } 
        if (gamepad1.y) {
          armrotate_desiredangle+=800 * (now_time-last_time);
        }
        if (gamepad1.x) {
          claw_desiredangle += 1 * (now_time-last_time);
        }
        if (gamepad1.b) {
          claw_desiredangle -= 1 * (now_time-last_time);
        }
        //Boundaries of the arm lengther
        if (arm_desiredangle < 0) { 
          //arm_desiredangle = 0;
        }
        if (arm_desiredangle > 100) {
          //arm_desiredangle = 100;
        }
        //Boundaries of the arm vertical rotation
        if (armrotate_desiredangle > 3000) {
          armrotate_desiredangle = 3000;
        }
        if (armrotate_desiredangle < 0) {
          armrotate_desiredangle = 0;
        }
        // Boundaries of the claw
        if (claw_desiredangle < 0) {
          claw_desiredangle = 0;
        }
        if (claw_desiredangle > 0.85) {
          claw_desiredangle = 0.85;
        }
        last_time = now_time;
        telemetry.addData("righttrigger", gamepad1.right_trigger);
        telemetry.addData("lefttrigger", gamepad1.left_trigger);
        telemetry.addData("leftstickx", gamepad1.left_stick_x);
        telemetry.addData("leftsticky", gamepad1.left_stick_y);
        telemetry.addData("rightstickx", gamepad1.right_stick_x);
        telemetry.addData("rightsticky", gamepad1.right_stick_y);
        telemetry.addData("arm_desiredangle", arm_desiredangle);
        telemetry.addData("armrotate_desiredangle", armrotate_desiredangle);
        telemetry.addData("claw_desiredangle", claw_desiredangle);
        telemetry.addData("arm_position", _pseudo_arm.getCurrentPosition());
        //telemetry.addData("arm_floor_distance", distance.getDistance(DistanceUnit.CM));
        telemetry.update();
        
        left_back_pow = gamepad1.left_stick_y;
        left_front_pow = gamepad1.left_stick_y;
        right_back_pow = gamepad1.right_stick_y;
        right_front_pow = gamepad1.right_stick_y;
        
        //The Triggers range from 0 to 1.
        //Is Right_Trigger held down enough?
        if (gamepad1.right_trigger > 0.05) {
          right_front_pow = gamepad1.right_trigger * 1;
          right_back_pow = gamepad1.right_trigger * -1;
          left_front_pow = gamepad1.right_trigger * -1;
          left_back_pow = gamepad1.right_trigger * 1;
        }
        
        //Is Left_Trigger held down enough?
        if (gamepad1.left_trigger > 0.05) {
          left_front_pow = gamepad1.left_trigger * 1;
          left_back_pow = gamepad1.left_trigger * -1;
          right_back_pow = gamepad1.left_trigger * 1;
          right_front_pow = gamepad1.left_trigger * -1;
        }
        right_front_pow = (float) (right_front_pow * 0.45);
        right_back_pow = (float) (right_back_pow * 0.45);
        left_front_pow = (float) (left_front_pow * 0.45);
        left_back_pow = (float) (left_back_pow * 0.61);
        
        //Set power of motors to their corresponding variables
        
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        leftback.setPower(left_back_pow);
        rightback.setPower(right_back_pow);
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        leftfront.setPower(left_front_pow);
        rightfront.setPower(right_front_pow);
        claw.setPosition(claw_desiredangle);
        
        _pseudo_arm.setTargetPosition(Help.degreesToTick(arm_desiredangle));
        armrotater.setTargetPosition(-Help.degreesToTick(armrotate_desiredangle));
        

        
        
        telemetry.update();
      }
    }
    
  }
}

class Help {
  public static int degreesToTick (int degrees) {
      int tickDegreeRatio = 5;

      return degrees/tickDegreeRatio;
  }
  public static int degreesToTick (float degrees) {
      int tickDegreeRatio = 5;

      return (int) degrees/tickDegreeRatio;
  }
  
  public static void limitDegrees(String name, float degrees) {
    if (name == "armrotater") {
      if (degrees > 3000) {
        degrees = 3000;
      }
      else if (degrees < 0) {
        degrees = 0;
      }
    ]
    else if (name == "_pseudo_arm") {
      if (arm_desiredangle < 0) { 
        arm_desiredangle = 0;
      }
      if (arm_desiredangle > 1400) {
        arm_desiredangle = 1400;
      }
    }
  }
}
