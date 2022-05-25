/*
Packages and Imports used for the code.
*/

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


/*
Code Starts Here.
*/

@TeleOp(name = "mechanumdrive (Blocks to Java)")
public class mechanumdrive extends LinearOpMode {

  private ElapsedTime     runtime = new ElapsedTime();
  private DcMotor wheel_leftback;
  private DcMotor wheel_leftfront;
  private DcMotor wheel_rightback;
  private DcMotor wheel_rightfront;
  private DcMotorEx arm_extender;
  private DcMotorEx arm_rotater;
  private Servo claw;
  private Servo claw_rotater;

  //private DistanceSensor distance;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double everything_universalscale = 1;
    double wheel_universalscale = 0.8;
    float wheel_leftback_pow;
    float wheel_leftfront_pow;
    float wheel_rightback_pow;
    float wheel_rightfront_pow;
    float arm_extender_desiredangle = 0;
    float arm_rotate_desiredangle = 0;
    double claw_grip_desiredangle = 0.28;
    double claw_rotate_desiredangle = 0.5;
    
    double last_time = runtime.seconds();
    double claw_rotate_last_time = runtime.seconds();
    
    boolean claw_rotating = false;


    wheel_leftback = hardwareMap.get(DcMotor.class, "left/back");
    wheel_leftfront = hardwareMap.get(DcMotor.class, "left/front");
    wheel_rightback = hardwareMap.get(DcMotor.class, "right/back");
    wheel_rightfront = hardwareMap.get(DcMotor.class, "right/front");
    arm_extender = hardwareMap.get(DcMotorEx.class, "_pseudo_arm");
    arm_rotater = hardwareMap.get(DcMotorEx.class, "armrotater");
    claw = hardwareMap.get(Servo.class, "claw");
    claw_rotater = hardwareMap.get(Servo.class, "clawrotater");
    //distance = hardwareMap.get(DistanceSensor.class, "Distance");

    
    
    
    
    
    //--These wheels are reversed for desired results--//
    wheel_leftback.setDirection(DcMotorSimple.Direction.REVERSE);
    wheel_leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
    //--//
    
    //--Set up the arm motors--//
    arm_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_rotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_extender.setTargetPosition(Help.degreesToTick(0));
    arm_rotater.setTargetPosition(Help.degreesToTick(0));
    arm_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm_rotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm_extender.setVelocity(750);
    arm_rotater.setVelocity(1200);
    //--//

    waitForStart();
    
    if (opModeIsActive()) {
      // Start the loop
      while (opModeIsActive()) {
        //now_time, the time since the start of the program and is used to find time differentials between loop iterations
        double now_time = runtime.seconds();
        
        ////----INPUTS----////
        
        if (gamepad1.left_bumper) {
          everything_universalscale = 0.4;
        }
        else {
          everything_universalscale = 1;
        }
        
        //dpad left/right
        if (gamepad1.dpad_right) {
          arm_extender_desiredangle-=500 * (now_time-last_time) * everything_universalscale;
        } 
        if (gamepad1.dpad_left) {
          arm_extender_desiredangle+=450 * (now_time-last_time)  * everything_universalscale;
        }
        if (gamepad1.dpad_down) {
          arm_rotate_desiredangle-=800 * (now_time-last_time) * everything_universalscale;
        } 
        if (gamepad1.dpad_up) {
          arm_rotate_desiredangle+=800 * (now_time-last_time) * everything_universalscale;
        }
        
        
        if (gamepad1.b) {
          claw_grip_desiredangle += 0.5 * (now_time-last_time) * everything_universalscale;
        }
        if (gamepad1.x) {
          claw_grip_desiredangle -= 0.5 * (now_time-last_time) * everything_universalscale;
        }
        
        if (gamepad1.y && ((now_time-claw_rotate_last_time) > 0.2 && !claw_rotating)) {
          claw_rotate_desiredangle -= 0.3 * everything_universalscale;
          claw_rotate_last_time = now_time;
          claw_rotating = true;
        }
        if (gamepad1.a && ((now_time-claw_rotate_last_time) > 0.2 && !claw_rotating)) {
          claw_rotate_desiredangle += 0.3 * everything_universalscale;
          claw_rotate_last_time = now_time;
          claw_rotating = true;
        }
        
        ////----BOUNDARIES----////
        
        
        if (arm_extender_desiredangle < 0) { 
          arm_extender_desiredangle = 0;
        }
        else if (arm_extender_desiredangle > 1600) {
          arm_extender_desiredangle = 1400;
        }
        
        //Boundaries of the arm vertical rotation
        if (arm_rotate_desiredangle > 3000) {
          arm_rotate_desiredangle = 3000;
        }
        else if (arm_rotate_desiredangle < 0) {
          arm_rotate_desiredangle = 0;
        }
        
        // Boundaries of the claw
        if (claw_grip_desiredangle < 0.28) {
          claw_grip_desiredangle = 0.28;
        }
        else if (claw_grip_desiredangle > 0.85) {
          claw_grip_desiredangle = 0.85;
        }
        
        // Boundaries of the claw rotate servo
        if (claw_rotate_desiredangle < 0.5 && ((now_time-claw_rotate_last_time) > 0.18) && claw_rotating) {
          claw_rotate_desiredangle = 0.5;
          claw_rotating = false;
        }
        if (claw_rotate_desiredangle > 0.5 && ((now_time-claw_rotate_last_time) > 0.18) && claw_rotating) {
          claw_rotate_desiredangle = 0.5;
          claw_rotating = false;
        }
        last_time = now_time; //To find time differentials between loops.
        
        ////----VARIABLE MONITORING----////
        
        telemetry.addData("righttrigger", gamepad1.right_trigger);
        telemetry.addData("lefttrigger", gamepad1.left_trigger);
        telemetry.addData("leftstickx", gamepad1.left_stick_x);
        telemetry.addData("leftsticky", gamepad1.left_stick_y);
        telemetry.addData("rightstickx", gamepad1.right_stick_x);
        telemetry.addData("rightsticky", gamepad1.right_stick_y);
        telemetry.addData("arm_desiredangle", arm_extender_desiredangle);
        telemetry.addData("armrotate_desiredangle", arm_rotate_desiredangle);
        telemetry.addData("claw_desiredangle", claw_grip_desiredangle);
        telemetry.addData("clawrotate_Desiredangle", claw_rotate_desiredangle);
        //telemetry.addData("claw_pos", clawrotate_position);
        //telemetry.addData("arm_position", _pseudo_arm.getCurrentPosition());
        //telemetry.addData("arm_floor_distance", distance.getDistance(DistanceUnit.CM));
        telemetry.update();
        
        ////----WHEEL DRIVING----////
        
        wheel_leftback_pow = gamepad1.left_stick_y;
        wheel_leftfront_pow = gamepad1.left_stick_y;
        wheel_rightback_pow = gamepad1.right_stick_y;
        wheel_rightfront_pow = gamepad1.right_stick_y;
        
        //The Triggers range from 0 to 1.
        //Is Right_Trigger held down enough?
        if (gamepad1.right_trigger > 0.05) {
          wheel_rightfront_pow = gamepad1.right_trigger * 1;
          wheel_rightback_pow = gamepad1.right_trigger * -1;
          wheel_leftfront_pow = gamepad1.right_trigger * -1;
          wheel_leftback_pow = gamepad1.right_trigger * 1;
        }
        
        //Is Left_Trigger held down enough?
        if (gamepad1.left_trigger > 0.05) {
          wheel_leftfront_pow = gamepad1.left_trigger * 1;
          wheel_leftback_pow = gamepad1.left_trigger * -1;
          wheel_rightback_pow = gamepad1.left_trigger * 1;
          wheel_rightfront_pow = gamepad1.left_trigger * -1;
        }
        wheel_rightfront_pow = (float) (wheel_rightfront_pow * 0.45 * wheel_universalscale * everything_universalscale);
        wheel_rightback_pow = (float) (wheel_rightback_pow * 0.45 * wheel_universalscale * everything_universalscale);
        wheel_leftfront_pow = (float) (wheel_leftfront_pow * 0.45 * wheel_universalscale * everything_universalscale);
        wheel_leftback_pow = (float) (wheel_leftback_pow * 0.61 * wheel_universalscale * everything_universalscale);
        
        //Set power of motors to their corresponding variables
        
        wheel_leftback.setPower(wheel_leftback_pow);
        wheel_rightback.setPower(wheel_rightback_pow);
        wheel_leftfront.setPower(wheel_leftfront_pow);
        wheel_rightfront.setPower(wheel_rightfront_pow);
        
        //Set position of arm and claw motors to their corresponding variables.
        
        claw.setPosition(claw_grip_desiredangle);
        claw_rotater.setPosition(claw_rotate_desiredangle);
        arm_extender.setTargetPosition(Help.degreesToTick(arm_extender_desiredangle));
        arm_rotater.setTargetPosition(-Help.degreesToTick(arm_rotate_desiredangle));
        

        
        
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
}
