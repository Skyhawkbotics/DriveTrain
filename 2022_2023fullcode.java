
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


/*
Code Starts Here.
*/

/*
Hello future programmer. My name is Kevin Vu and initially wrote this code.
It is not very readable and some of the design decisions I made are very questionable.

| For the movement of the claw, look at the gamepadInputHandling() claw section and the clawMove() section.
| now_time-last_time is used to find time inbetween loops and normalize degrees per loop to degrees per second.
*/

@TeleOp(name = "2022-2023fullcode")
public class mechanumdrive extends LinearOpMode {
  //Clock
  private ElapsedTime     runtime = new ElapsedTime();

  //Create Motor Variables
  private DcMotor whl_LB;
  private DcMotor whl_LF;
  private DcMotor whl_RB;
  private DcMotor whl_RF;
  private DcMotorEx arm_EXT;
  private DcMotorEx arm_ROT;
  private Servo claw_GRIP;
  private Servo wrist_ROT;

  
  //time at which the claw rotates for per movement. Modified when restarting the robot.
  //double CLAW_ROTATE_TIME = 0.18;
  
  //double everything_universalscale = 1;  //Multiplier for angle/power
  //double wheel_universalscale = 0.8; //Multiplier for power for wheels
  //double wheel_equalizerscale = 0; //How much the difference between two sides of the wheel should be evened by (0-1)
  float whl_LB_percent;
  float whl_LF_percent;
  float whl_RB_percent;
  float whl_RF_percent;
  float arm_EXT_angle = 0; // 0 to 1300 | retracted to fully extended
  float arm_ROT_angle = 0; // 0 to 3500 | lowered to fully raised
  double claw_GRIP_angle = 0.28; // 0.28 to 0.85 | closed to fully opened
  double wrist_ROT_percent = 0.5; // >0.5 to <0.5 | move up or move down
  
  double wrist_ROT_pos = 0; // Increases or decreases based on how much movement the claw makes | Utilized for finding how much to readjust the claw by to reset it.
  
  double last_time = runtime.seconds(); //Used to find how much time has elapsed per iteration in the runtime loop.
  double claw_rotate_last_time = runtime.seconds(); //Last time the claw moved
  double reset_last_time = runtime.seconds(); //Last time the robot has reset
  
  boolean stopreset_soon = false; //Is the robot trying to reset all the motors? (Except wheels)
  
  boolean claw_rotating = false; //Is the claw moving?


  //private DistanceSensor distance;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    //Initalize Motors and Servos
    whl_LB = hardwareMap.get(DcMotor.class, "left/back");
    whl_LF = hardwareMap.get(DcMotor.class, "left/front");
    whl_RB = hardwareMap.get(DcMotor.class, "right/back");
    whl_RF = hardwareMap.get(DcMotor.class, "right/front");
    arm_EXT = hardwareMap.get(DcMotorEx.class, "arm_extender");
    arm_ROT = hardwareMap.get(DcMotorEx.class, "arm_rotater");
    claw_grip = hardwareMap.get(Servo.class, "claw_grip");
    wrist_ROT = hardwareMap.get(Servo.class, "wrist_ROT");
    
    //--These wheels are reversed for desired results--//
    whl_LB.setDirection(DcMotorSimple.Direction.REVERSE);
    whl_LF.setDirection(DcMotorSimple.Direction.REVERSE);
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
    /*
    */
    waitForStart();
    
    if (opModeIsActive()) {
      // Start the loop
      while (opModeIsActive()) {
        //now_time, the time since the start of the program and is used to find time differentials between loop iterations
        double now_time = runtime.seconds();
        
        ////----INPUTS----////
        gamepadInputHandling();
        
        
        
        //--All things related to resetting the motors--//
        if (now_time - reset_last_time > 4 && stopreset_soon) {
          stopreset_soon = false;
        }
        else if (now_time - reset_last_time < 4 && stopreset_soon){
          arm_extender_desiredangle-=500 * (now_time-last_time);
          arm_rotate_desiredangle-=300 * (now_time-last_time) * everything_universalscale;
        }
        
        if (gamepad1.start && !stopreset_soon) {
          stopreset_soon = true;
          
          claw_grip_desiredangle = 0.28;
          clawMove(1,claw_rotate_position, now_time);
          claw_rotate_position = 0;
          
          reset_last_time = runtime.seconds();
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
        telemetry.addData("clawrotate_position", claw_rotate_position);
        telemetry.addData("stopreset", stopreset_soon);
        telemetry.update();
        
        ////----WHEEL DRIVING----////
        
        float difference = (gamepad1.left_stick_y - gamepad1.right_stick_y) * (float) wheel_equalizerscale;
        
        whl_LB_percent = gamepad1.left_stick_y -difference;
        whl_LF_percent = gamepad1.left_stick_y - difference;
        whl_RB_percent = gamepad1.right_stick_y + difference;
        whl_RF_percent = gamepad1.right_stick_y + difference;
        
        
        
        //The Triggers range from 0 to 1.
        //Is Right_Trigger held down enough?
        if (gamepad1.right_trigger > 0.05) {
          whl_RF_percent = gamepad1.right_trigger * 1;
          whl_RB_percent = gamepad1.right_trigger * -1;
          whl_LF_percent = gamepad1.right_trigger * -1;
          whl_LB_percent = gamepad1.right_trigger * 1;
        }
        
        //Is Left_Trigger held down enough?
        if (gamepad1.left_trigger > 0.05) {
          whl_LF_percent = gamepad1.left_trigger * 1;
          whl_LB_percent = gamepad1.left_trigger * -1;
          whl_RB_percent = gamepad1.left_trigger * 1;
          whl_RF_percent = gamepad1.left_trigger * -1;
        }
        whl_RF_percent = (float) (whl_RF_percent * 0.45 * wheel_universalscale);
        whl_RB_percent = (float) (whl_RB_percent * 0.45 * wheel_universalscale);
        whl_LF_percent = (float) (whl_LF_percent * 0.45 * wheel_universalscale);
        whl_LB_percent = (float) (whl_LB_percent * 0.61 * wheel_universalscale);
        
        //Set power of motors to their corresponding variables
        
        whl_LB.setPower(whl_LB_percent);
        whl_RB.setPower(whl_RB_percent);
        whl_LF.setPower(whl_LF_percent);
        whl_RF.setPower(whl_RF_percent);
        
        //Set position of arm and claw motors to their corresponding variables.
        
        claw.setPosition(claw_grip_desiredangle);
        claw_rotater.setPosition(claw_rotate_desiredangle);
        arm_extender.setTargetPosition(-Help.degreesToTick(arm_extender_desiredangle));
        arm_rotater.setTargetPosition(-Help.degreesToTick(arm_rotate_desiredangle));
        

        
        
        telemetry.update();
      }
    }
    
    
    
  }
  
  public void gamepadInputHandling() {
    /*
    if (gamepad1.left_bumper) {
      everything_universalscale = 0.4;
      wheel_universalscale = 0.3;
    }
    else {
      everything_universalscale = 1;
      wheel_universalscale = 0.8;
    }

    if (gamepad1.right_bumper) {
      wheel_equalizerscale = 0.3;
    }
    else {
      wheel_equalizerscale = 0;
    }*/

    //dpad left/right wrist rotation

    //dpad up/down claw open/close

    // Y A arm ROT up down

    // B X arm EXT forward back

    if (gamepad1.dpad_right) {
      arm_extender_desiredangle-=500 * (now_time-last_time) * everything_universalscale;
    } 
    else if (gamepad1.dpad_left) {
      arm_extender_desiredangle+=450 * (now_time-last_time)  * everything_universalscale;
    }
    if (gamepad1.dpad_down) {
      arm_rotate_desiredangle-=800 * (now_time-last_time) * everything_universalscale;
    } 
    else if (gamepad1.dpad_up) {
      arm_rotate_desiredangle+=800 * (now_time-last_time) * everything_universalscale;
    }


    if (gamepad1.b) {
      claw_grip_desiredangle += 0.5 * (now_time-last_time) * everything_universalscale;
    }
    else if (gamepad1.x) {
      claw_grip_desiredangle -= 0.5 * (now_time-last_time) * everything_universalscale;
    }

    if (gamepad1.y && ((now_time-claw_rotate_last_time) > 0.2 && !claw_rotating)) {
      clawMove(1,1, now_time);
    }
    else if (gamepad1.a && ((now_time-claw_rotate_last_time) > 0.2 && !claw_rotating)) {
      clawMove(-1,1, now_time);
    }

    
    ////----BOUNDARIES----////


    if (arm_extender_desiredangle < 0) { 
      arm_extender_desiredangle = 0;
    }
    else if (arm_extender_desiredangle > 1300) {
      arm_extender_desiredangle = 1300;
    }

    //Boundaries of the arm vertical rotation
    if (arm_rotate_desiredangle > 3500) {
      arm_rotate_desiredangle = 3500;
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
    if (claw_rotate_desiredangle < 0.5 && ((now_time-claw_rotate_last_time) > CLAW_ROTATE_TIME) && claw_rotating) {
      claw_rotate_desiredangle = 0.5;
      claw_rotating = false;
    }
    if (claw_rotate_desiredangle > 0.5 && ((now_time-claw_rotate_last_time) > CLAW_ROTATE_TIME) && claw_rotating) {
      claw_rotate_desiredangle = 0.5;
      claw_rotating = false;
    }
  }
  
  public void clawMove(int mult, double iterations, double now_time){
    if (iterations == 1) {
      claw_rotate_desiredangle -= 0.3 * everything_universalscale * mult;
    }
    else {
      claw_rotate_desiredangle += 0.3 * everything_universalscale * Help.numSign(iterations);
    }
    claw_rotate_last_time = now_time;
    claw_rotating = true;
    
    
    if (iterations == 1) {
     claw_rotate_position += 1 * mult * everything_universalscale;
    }
    
    CLAW_ROTATE_TIME = 0.18 * iterations;
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
  
  public static double numSign (double num) {
    if (num >= 0) {
      return 1.01;
    }
    else {
      return -1.01;
    }
  }
}
