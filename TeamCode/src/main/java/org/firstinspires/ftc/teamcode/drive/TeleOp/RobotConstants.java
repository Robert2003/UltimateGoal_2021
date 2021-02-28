package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotConstants
{
    public DcMotor leftFront = null, leftRear = null, rightFront = null, rightRear = null;

    public DcMotor intake1   = null, intake2 = null;
    public DcMotorEx flyWheel  = null;
    public DcMotor wobbleArm = null;

    public Servo servo, wobbleServo;

    HardwareMap hardwareMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public RobotConstants(){}

    public void init(HardwareMap ahwMap)
    {
        hardwareMap = ahwMap;

        leftFront  = hardwareMap.get(DcMotor.class, "fl_motor");
        rightFront = hardwareMap.get(DcMotor.class, "fr_motor");
        leftRear   = hardwareMap.get(DcMotor.class, "bl_motor");
        rightRear  = hardwareMap.get(DcMotor.class, "br_motor");

        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        intake1   = hardwareMap.get(DcMotor.class, "intake1");
        intake2   = hardwareMap.get(DcMotor.class, "intake2");
        flyWheel  = hardwareMap.get(DcMotorEx.class, "flyWheel");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");

        intake2.setDirection(DcMotor.Direction.REVERSE);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        servo       = hardwareMap.get(Servo.class, "servo");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        servo.setPosition(0.5);
    }
 }

