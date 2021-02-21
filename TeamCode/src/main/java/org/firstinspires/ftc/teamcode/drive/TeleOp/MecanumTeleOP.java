package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.awt.Robot;

import static java.lang.Math.*;

@TeleOp(name="MecanumTeleOP", group="Iterative Opmode")
//@Disabled
public class MecanumTeleOP extends OpMode
{
    private RobotConstants robot = new RobotConstants();

    private ElapsedTime runtime = new ElapsedTime();

    double rearLeftPower;
    double rearRightPower;
    double frontLeftPower;
    double frontRightPower;

    double flyWheelPower;

    double drive, strafe, rotate;

    double powerLatchingUp, powerLatchingDown;

    boolean supress2 = false;

    @Override
    public void init()
    {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start()
    {
        runtime.reset();
    }

    @Override
    public void loop()
    {
        /*Acceleration angle = imu.getGravity();*/
        telemetry.addData("x1", robot.leftFront.getCurrentPosition());
        telemetry.addData("x2", robot.rightFront.getCurrentPosition());
        telemetry.addData("x3", robot.rightRear.getCurrentPosition());
        telemetry.addData("x4", robot.leftRear.getCurrentPosition());
        telemetry.addData("wobble", robot.wobbleArm.getCurrentPosition());
        telemetry.update();

        if (gamepad2.y) {
            robot.intake1.setPower(1);
            robot.intake2.setPower(1);
        } else if (gamepad2.a) {
            robot.intake1.setPower(-1);
            robot.intake2.setPower(-1);
        } else if (gamepad2.x) {
            robot.intake1.setPower(0);
            robot.intake2.setPower(0);
        }

        powerLatchingUp = gamepad2.left_trigger;
        powerLatchingDown = -gamepad2.right_trigger;

        if((powerLatchingUp==0 && powerLatchingDown==0))
        {
            robot.wobbleArm.setTargetPosition(robot.wobbleArm.getCurrentPosition());
            robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleArm.setPower(1);
        }
        else
            robot.wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(powerLatchingDown!=0 || powerLatchingUp!=0)
            robot.wobbleArm.setPower(powerLatchingUp+powerLatchingDown);

        if(gamepad2.dpad_left)
            robot.wobbleServo.setPosition(1);
        if(gamepad2.dpad_right)
            robot.wobbleServo.setPosition(0);
        if(gamepad2.dpad_down)
            robot.wobbleServo.setPosition(0.5);

        if(gamepad2.right_bumper)
            robot.flyWheel.setPower(0.85f);
        else
            robot.flyWheel.setPower(0f);

        drive  = gamepad1.right_stick_y;
        strafe = -gamepad1.right_stick_x;
        rotate = -gamepad1.right_trigger + gamepad1.left_trigger;

        rearLeftPower   = -strafe + drive + rotate;
        frontLeftPower  = strafe + drive + rotate;
        rearRightPower  = strafe + drive - rotate;
        frontRightPower = -strafe + drive - rotate;

        rearLeftPower   = Range.clip(rearLeftPower, -1.0, 1.0);
        rearRightPower  = Range.clip(rearRightPower, -1.0, 1.0);
        frontLeftPower  = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);

        if (gamepad1.right_bumper) {
            rearLeftPower *= 0.3;
            frontLeftPower *= 0.3;
            rearRightPower *= 0.3;
            frontRightPower *= 0.3;
        }


        if(gamepad1.left_bumper)
            supress2 = !supress2;

        if(supress2)
        {
            rearLeftPower *= 0.6;
            frontLeftPower *= 0.6;
            rearRightPower *= 0.6;
            frontRightPower *= 0.6;
        }

        robot.leftFront.setPower(frontLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.leftRear.setPower(rearLeftPower);
        robot.rightRear.setPower(rearRightPower);

        if(gamepad2.left_bumper)
        {
            if(runtime.seconds() > 1.6)
            {
                runtime.reset();
                robot.servo.setPosition(0);
            }
            else if(runtime.seconds() > 0.8)
                robot.servo.setPosition(0.5);
        }
        else
            robot.servo.setPosition(0.5);
    }

    @Override
    public void stop()
    {

    }

    public double speed(double X, double Y)
    {
        double alfa = 37.0;
        double deltaY = Y-0.15;
        double numarator = 4.9*X*X;
        double numitor = (tan(toRadians(alfa))*X-deltaY)*cos(toRadians(alfa))*cos(toRadians(alfa));
        double cat = numitor/numarator;
        return sqrt(cat);
    }

}
