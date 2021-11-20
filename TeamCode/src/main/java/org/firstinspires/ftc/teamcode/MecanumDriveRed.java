package FreightFrenzy.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mecanum Drive Red", group="TeleOpMecanum")
public class MecanumDriveRed extends LinearOpMode {
   @Override
   public void runOpMode() throws InterruptedException {
	  // Declare our motors
	  // Make sure your ID's match your configuration
	  DcMotor motorFrontLeft = hardwareMap.dcMotor.get("front_left");
	  DcMotor motorBackLeft = hardwareMap.dcMotor.get("back_left");
	  DcMotor motorFrontRight = hardwareMap.dcMotor.get("front_right");
	  DcMotor motorBackRight = hardwareMap.dcMotor.get("back_right");
	  DcMotor arm1 = hardwareMap.dcMotor.get("arm1");
	  DcMotor carasel = hardwareMap.dcMotor.get("duckies");
	  Servo wrist1 = hardwareMap.servo.get("wrist");
	  Servo grabber = hardwareMap.servo.get("grabber");
	  

	  // Reverse the right side motors
	  // Reverse left motors if you are using NeveRests

	  motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
	  motorBackRight.setDirection(DcMotor.Direction.REVERSE);

		// Set motors to brake when not in use
		motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
	  waitForStart();

	  if (isStopRequested()) return;

	  while (opModeIsActive()) {
	  	
	  	
	  	 double arm1pwr = -gamepad2.left_stick_y;
		 double rx = -gamepad1.right_stick_x; // Remember, this is reversed!
		 double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
		 double y = gamepad1.left_stick_y;
		 

		 double frontLeftPower = y + x + rx;
		 double backLeftPower = y - x + rx;
		 double frontRightPower = y - x - rx;
		 double backRightPower = y + x - rx;

		 // Put powers in the range of -1 to 1 only if they aren't already
		 // Not checking would cause us to always drive at full speed
		 if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
			Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1) {
			// Find the largest power
			double max = 0;
			max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
			max = Math.max(Math.abs(frontRightPower), max);
			max = Math.max(Math.abs(backRightPower), max);

			// Divide everything by max (it's positive so we don't need to worry
			// about signs)
			frontLeftPower /= max;
			backLeftPower /= max;
			frontRightPower /= max;
			backRightPower /= max;
		 }
		if(gamepad1.right_bumper){
		 motorFrontLeft.setPower(frontLeftPower * 0.25);
		 motorBackLeft.setPower(backLeftPower * 0.25);
		 motorFrontRight.setPower(frontRightPower * 0.25);
		 motorBackRight.setPower(backRightPower * 0.25);
		 telemetry.addLine("Speed one quarter");
		 telemetry.update();
		}
		else if(gamepad1.left_bumper){
			motorFrontLeft.setPower(frontLeftPower * 0.75);
			motorBackLeft.setPower(backLeftPower * 0.75);
			motorFrontRight.setPower(frontRightPower * 0.75);
			motorBackRight.setPower(backRightPower * 0.75);
			telemetry.addLine("Speed 3/4");
		 telemetry.update();
		}else{
		 motorFrontLeft.setPower(frontLeftPower);
		 motorBackLeft.setPower(backLeftPower);
		 motorFrontRight.setPower(frontRightPower);
		 motorBackRight.setPower(backRightPower);
		 telemetry.addLine("Speed full");
		 telemetry.update();
		}
		
		
		if(gamepad2.a){
			grabber.setPosition(0.7);
			telemetry.addLine("position 20");
			telemetry.update();
		} else if(gamepad2.b){
			grabber.setPosition(0.45);
			telemetry.addLine("position 30");
			telemetry.update();
		} else if(gamepad2.y){
			grabber.setPosition(0.1);
			telemetry.addLine("position 50");
			telemetry.update();
		}
		
		//set wrist position to wrist position plus gamepad2.right_joystick_y
		wrist1.setPosition(1);
		
		
		if(gamepad2.right_bumper){
		 arm1.setPower(arm1pwr * 0.25);
		}
		else if(gamepad2.left_bumper){
			arm1.setPower(arm1pwr * 0.5);
		}else{
		 arm1.setPower(arm1pwr * 0.8);
		}
		
		if(gamepad1.x || gamepad2.x){
			carasel.setPower(0.5);
		}else{
			carasel.setPower(0);
		}
	  }
   }
   }
