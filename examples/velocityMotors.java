//Init
private DcMotorEx arm;
  arm = hardwareMap.get(DcMotorEx.class, "arm");
  arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//TeleOp
if (arm.getCurrentPosition() < 500 && gamepad1.Y) {
  arm.setVelocity(100); //Can be positive or negative
}
else if (arm.getCurrentPosition() > 0 && !gamepad1.Y) {
  arm.setVelocity(-100);
}
