package org.firstinspires.ftc.teamcode.ControlFreaks;

/**
 * Created by adevries on 11/6/2015.
 */

import android.media.AudioManager;
import android.media.ToneGenerator;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;

//import com.qualcomm.ftcrobotcontroller.opmodes.ColorSensorDriver;

public class CFPushBotHardware {

    private String config_motor_leftdrive = "left_drive";
    private String config_motor_rightdrive = "right_drive";
    private String config_motor_rackpinion = "rackpinion";
    private String config_motor_lifter = "lifter";
    private String config_servo_hand = "hand";
    private String config_servo_wrist = "wrist";
    private String config_servo_blockgrabber = "blockgrabber";
    private String config_servo_elbow = "elbow";
    private String config_servo_shoulder = "shoulder";
    private String config_dim = "dim";
    private String config_i2c_gyro = "gyro";
    private String config_i2c_led7seg = "ledseg";
    private String config_i2c_colorsensor = "color";
    /*
        Motor Encoder Vars
     */
    /*
        AndyMark 20 560 Pulses per 360
        AndyMark 40 1120 Pulses per 360
        AndyMark 60 1680 Pulses per 360
     */
/*
    //Andy Mark 20 motors
    private static final float v_drive_turn_ticks_per_degree = 5.0f;
    private static final double v_drive_inches_ticksPerInch = 42.00;
    //This is the value used to stop before reaching target to account for delay in stop command being processed
    private static final int v_drive_inches_ticksStop = 300;
    //ticks before target to slow to slowdown 1 speed hapens before slowdown 2
    private static final int v_drive_inches_ticksSlowDown1 = 1000;
    private static final float v_turn_motorspeed = .3f;
    private static final float v_turn_motorspeed_slow = .2f;
    private static final int v_turn_degrees_heading_drift_error = 5;

    //old treads
    //private static final double driveInches_ticksPerInch = 182.35;

    //new Treads
    //ticks before target to slow to slowdown 2 speed
    //private static final double v_drive_inches_ticksSlowDown2 = 1000;
    private static final float v_drive_power_slowdown1 = .2f;
    //private static final float v_drive_power_slowdown2 = .30f;
    private static final float v_drive_power_slowdown1 = .2f;
*/

    //Andy Mark 40 motors
    //private static final float v_drive_turn_ticks_per_degree = 10.83f;  //10
    //private static final double v_drive_inches_ticksPerInch = 88.00;
    private float v_drive_turn_ticks_per_degree = 10.83f;  //10
    private double v_drive_inches_ticksPerInch = 88.00;

    //This is the value used to stop before reaching target to account for delay in stop command being processed
    private static final int v_drive_inches_ticksStop = 150;
    //ticks before target to slow to slowdown 1 speed happens before slowdown 2
    //private static final int v_drive_inches_ticksSlowDown1 = 1000;
    private float v_drive_power = 1f;
    private float v_drive_power_reverse = .5f;
    private float v_drive_power_slowdown = .3f;
    //Motor speed for turns
    //Am40
    //private static final float v_turn_motorspeed = .8f;
    //private static final float v_turn_motorspeed_slow = .5f;

    private  float v_turn_motorspeed = .5f;
    private  float v_turn_motorspeed_slow = .25f;


    //old treads
    //private static final double driveInches_ticksPerInch = 182.35;

    //new Treads
    //ticks before target to slow to slowdown 2 speed
    //private static final double v_drive_inches_ticksSlowDown2 = 1000;

    //private static final float v_drive_power_slowdown2 = .30f;


    private boolean v_debug = true;  //set this to false to prevent writing to log makes loop lots shorter

    //We Increment the v_loop_ticks each time through our loop
    private long v_loop_ticks = 0;
    // each time through the loop we check to see if v_loop_ticks % v_loop_ticks_slow_count == 0 is so then slow loop = true
    private int v_loop_ticks_slow_count = 60;  //20
    private boolean v_loop_ticks_slow = false;
    private boolean v_drive_use_slowdown = true;
    private int v_drive_inches_slowdown = 24;  //slowdown inches before target
    private int v_drive_inches_slowdown_ticks; //do not set this the inches above is used to calc this on init
    //old treads
    //private static final float v_turn_ticks_per_degree = 18.8f;
    //new treads


    //Global Vars to the class
    private static final double ServoErrorResultPosition = -0.0000000001;

    private DcMotor v_motor_lifter;
    private static final double v_motor_lifter_power = 0.7;

    //4.66666  ticks per degree on 1680 per 360
    private DcMotor v_motor_rackpinion;
    private static final double v_motor_rackpinion_Speed = 1.0f;
    private static final double v_motor_rackpinion_SpeedSlowDown = 0.5f;
    private static final int v_motor_rackpinion_ExtendTicks = 840;
    private static final int v_motor_rackpinion_ExtendSlowdownTicks = 300;
    private int v_motor_rackpinion_Position = 0;



    private Servo v_servo_pushbutton_left;
    private static final double v_servo_pushbutton_left_MinPosition = 0.00;
    private static final double v_servo_pushbutton_left_MaxPosition = 1.00;
    private double v_servo_pushbutton_left_position = 1.0D;  //init arm elbow Position


    private Servo v_servo_pushbutton_right;
    private static final double v_servo_pushbutton_right_MinPosition = 0.00;
    private static final double v_servo_pushbutton_right_MaxPosition = 1.00;
    private double v_servo_pushbutton_right_position = 0.0D;  //init arm elbow Position

    private Servo v_servo_blockgrabber;
    private static final double v_servo_blockgrabber_MinPosition = 0.00;
    private static final double v_servo_blockgrabber_MaxPosition = 0.45;
    private static final double v_servo_blockgrabber_MiddlePosition = 0.23;
    private double v_servo_blockgrabber_position = 0.23D;  //init arm elbow Position
    boolean v_servo_blockgrabber_is_extended = false;

    private Servo v_servo_elbow;
    private static final double v_servo_elbow_MinPosition = 0.00;
    private static final double v_servo_elbow_MaxPosition = 0.50;
    private double v_servo_elbow_position = 0.00D;  //init arm elbow Position
    boolean v_servo_elbow_is_extended = false;

    private Servo v_servo_shoulder;
    private static final double v_servo_shoulder_MinPosition = 0.00;
    private static final double v_servo_shoulder_MaxPosition = 0.50;
    private double v_servo_shoulder_position = 0.00D;  //init arm shoulder Position
    boolean v_servo_shoulder_is_extended = false;

    //Legecy Color Sensor
  /*  private ColorSensor v_sensor_colorLegecy;
    private final static String v_sensor_colorLegecy_name="color1";
    private boolean v_sensor_colorLegecy_led_enabled = false;
    // v_sensor_color_hsvValues is an array that will hold the hue, saturation, and value information.
    private float v_sensor_colorLegecy_hsvValues[] = {0F,0F,0F};
    // values is a reference to the v_sensor_color_hsvValues array.
    private final float v_sensor_colorLegecy_values[] = v_sensor_colorLegecy_hsvValues;
    private int v_sensor_colorLegecy_rgbValues[] = {0,0,0,0};
*/
    //Adafruit RGB Sensor
    private ColorSensor v_sensor_color_i2c;
    private static final int v_sensor_color_i2c_led_pin = 1;
    // bEnabled represents the state of the LED.
    private boolean v_sensor_color_i2c_led_enabled = false;
    //red, green, blue, alpha
    private int v_sensor_color_i2c_rgbaValues[]= {0,0,0,0};
    //we read the values in the loop only if the sensor is enabled as they take resourses
    private boolean v_sensor_color_i2c_enabled = false;

    //Legecy OSD Sensor
    //private OpticalDistanceSensor v_sensor_odsLegecy;
    //private boolean v_sensor_odsLegecy_enabled = false;

    //Legecy Light Sensor
   /* private LightSensor v_sensor_lightLegecy;
    private static final String v_sensor_lightLegecy_name = "light1";
    private boolean v_sensor_lightLegecy_enabled = false;

    private static final String v_sensor_ultraLegecy_name = "ultra1";
    private UltrasonicSensor v_sensor_ultraLegecy;
    private int v_sensor_ultraLegecy_ticksPerRead = 20;
    private double v_sensor_ultraLegecy_distance;
*/
    //Modern Robotics gyro1
    ModernRoboticsI2cGyro v_sensor_gyro_mr;
    GyroSensor v_sensor_gyro;
    //private int v_sensor_gyro_x, v_sensor_gyro_y, v_sensor_gyro_z = 0;
    //private int v_sensor_gyro_heading = 0;

    //Tone Generator to make noise
    ToneGenerator v_tone_generator;
    AudioEffects v_audio_effects;
// (tone type, tone duration in ms)
// from a list of predefined tone types
    LinearOpMode opMode;

//    private LED v_led_heartbeat;
//    private boolean v_led_heartbeat_enabled = true;
//    private  static final int v_led_heartbeat_tickPerToggle = 20;
    //private int v_led_heartbeat_ticks = 0;

    private DeviceInterfaceModule v_dim;

    private AdafruitLEDBackpack7Seg v_ledseg;

    // I2C wouldn't work with Modern Robotic Controller for some reason moved to
    // digital pins to send color and mode
//    private static final boolean v_neopixels_use_i2c = true;
//    private ArduinoI2CNeopixels v_neopixels;
//    private static final int v_neopixel_modechange_pin = 7;
//    private static final int v_neopixel_blue_pin = 6;
//    private static final int v_neopixel_green_pin = 5;
//    private static final int v_neopixel_red_pin = 4;

    private DcMotor v_motor_left_drive;
    private DcMotor v_motor_right_drive;

    /**
     * Indicate whether a message is a available to the class user.
     */
    private boolean v_warning_generated = false;

    /**
     * Store a message to the user if one has been generated.
     */
    private String v_warning_message = "No Errors";


    public CFPushBotHardware()
    {
    }



    //--------------------------------------------------------------------------
    //
    // init
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    public void init (LinearOpMode ahOpmode)

    {
        //
        // Use the hardwareMap to associate class members to hardware ports.
        //
        // Note that the names of the devices (i.e. arguments to the get method)
        // must match the names specified in the configuration file created by
        // the FTC Robot Controller (Settings-->Configure Robot).

        opMode = ahOpmode;
        v_warning_generated = false;
        v_warning_message = "Can't map; ";

        //
        //Connect the Core Interface Device or Dim
        try {

            // set up the hardware devices we are going to use
            v_dim = opMode.hardwareMap.deviceInterfaceModule.get(config_dim);


        }catch (Exception p_exeception)
        {
            debugLogException(config_dim,"missing",p_exeception);

            v_dim = null;
        }


        try {
            // get a reference to our GyroSensor object.
            v_sensor_gyro = opMode.hardwareMap.gyroSensor.get(config_i2c_gyro);
            // calibrate the gyro.
            v_sensor_gyro.calibrate();
            // make sure the gyro is calibrated.
            while (v_sensor_gyro.isCalibrating())  {
                sleep(50);
            }
            //v_sensor_gyro_mr = (ModernRoboticsI2cGyro) v_sensor_gyro;
            //v_sensor_gyro_mr.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
            sleep(100);
            v_sensor_gyro.resetZAxisIntegrator();
            sleep(200);
            //v_sensor_gyro_heading = v_sensor_gyro.getHeading();
            set_second_message("Gyro isCalibrated H:" + v_sensor_gyro.getHeading() );
        }catch(Exception p_exeception){
            debugLogException(config_i2c_gyro,"missing",p_exeception);
            v_sensor_gyro = null;
        }

        //
        // Connect the drive wheel motors.
        //
        // The direction of the right motor is reversed, so joystick inputs can
        // be more generically applied.
        //
        try
        {
            v_motor_left_drive = opMode.hardwareMap.dcMotor.get (config_motor_leftdrive);

            //v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            debugLogException(config_motor_leftdrive,"missing",p_exeception);
            v_motor_left_drive = null;
        }

        try
        {
            v_motor_right_drive = opMode.hardwareMap.dcMotor.get (config_motor_rightdrive);

            v_motor_right_drive.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            debugLogException(config_motor_rightdrive, "missing", p_exeception);
            v_motor_right_drive = null;
        }
        try {
            int counter = 0;
            reset_drive_encoders();
            while (counter < 10 && have_drive_encoders_reset() == false){
                counter++;
                sleep(100);
                debugLogException("init", "waiting on  rest_drive_encoders() complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
            }
            run_using_encoders();
            debugLogException("init", "run_using_encoders() and rest_drive_encoders() complete", null);
        }catch (Exception p_exeception)
        {
            debugLogException("run_using encoders", "error", p_exeception);
            v_motor_right_drive = null;
        }

        try{
            v_tone_generator = new ToneGenerator(AudioManager.STREAM_RING, ToneGenerator.MAX_VOLUME);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 500);
            /*sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 500);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 500);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 500);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_4, 500);
            */
            sleep(200);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_7, 500);
            sleep(200);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 500);
        }catch (Exception p_exeception)
        {
            debugLogException("toneGenerator", "missing", p_exeception);

            v_tone_generator = null;
        }

        try{
            v_audio_effects = new AudioEffects();
        }catch (Exception p_exeception)
        {
            debugLogException("AudioEffects", "missing", p_exeception);

            v_audio_effects = null;
        }
        try{
            if(v_dim != null) {
                // set the digital channel to output mode.
                // remember, the Adafruit sensor is actually two devices.
                // It's an I2C sensor and it's also an LED that can be turned on or off.
                v_dim.setDigitalChannelMode(v_sensor_color_i2c_led_pin, DigitalChannelController.Mode.OUTPUT);
                // get a reference to our ColorSensor object.
                v_sensor_color_i2c = opMode.hardwareMap.colorSensor.get(config_i2c_colorsensor);
                // turn the LED on in the beginning, just so user will know that the sensor is active.
                v_dim.setDigitalChannelState(v_sensor_color_i2c_led_pin, v_sensor_color_i2c_led_enabled);
            }
        } catch (Exception p_exeception)
        {
            debugLogException(config_i2c_colorsensor, "missing", p_exeception);
            v_sensor_color_i2c = null;
        }

        //
        // Connect the left_push servo.
        //
        try
        {
            v_servo_pushbutton_left = opMode.hardwareMap.servo.get(config_servo_hand);
            v_servo_pushbutton_left.setPosition (v_servo_pushbutton_left_position);
        }
        catch (Exception p_exeception)
        {
            debugLogException(config_servo_hand, "missing", p_exeception);
            v_servo_pushbutton_left = null;
        }
        //
        // Connect the right_push servo.
        //
        try
        {
            v_servo_pushbutton_right = opMode.hardwareMap.servo.get(config_servo_wrist);
            v_servo_pushbutton_right.setPosition (v_servo_pushbutton_right_position);
        }
        catch (Exception p_exeception)
        {
            debugLogException(config_servo_wrist, "missing", p_exeception);
            v_servo_pushbutton_right = null;
        }

        //
        // Connect the blockgrabber servo.
        //
        try
        {
            v_servo_blockgrabber = opMode.hardwareMap.servo.get(config_servo_blockgrabber);
            v_servo_blockgrabber.setPosition (v_servo_blockgrabber_position);
            //v_server_blockgrabber_is_extended = false;
        }
        catch (Exception p_exeception)
        {
            debugLogException(config_servo_blockgrabber, "missing", p_exeception);
            v_servo_blockgrabber = null;
        }




        //
        // Connect the elbow servo.
        //
        try
        {
            v_servo_elbow = opMode.hardwareMap.servo.get(config_servo_elbow);
            v_servo_elbow.setPosition (v_servo_elbow_position);
        }
        catch (Exception p_exeception)
        {
            debugLogException(config_servo_elbow, "missing", p_exeception);
            v_servo_elbow = null;
        }


        //
        // Connect the shoulder servo.
        //
        try
        {
            v_servo_shoulder = opMode.hardwareMap.servo.get(config_servo_shoulder);
            v_servo_shoulder.setPosition (v_servo_shoulder_position);
        }
        catch (Exception p_exeception)
        {
            debugLogException(config_servo_shoulder, "missing", p_exeception);
            v_servo_shoulder = null;
        }



        try
        {
            v_motor_rackpinion = opMode.hardwareMap.dcMotor.get (config_motor_rackpinion);
            v_motor_rackpinion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_rackpinion.setDirection(DcMotor.Direction.REVERSE);
            v_motor_rackpinion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            while(v_motor_rackpinion.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
//                sleep(10);
//            }
            int counter = 0;
            while (counter < 10 && v_motor_rackpinion.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER){
                counter++;
                sleep(10);
                debugLogException("init", "waiting on rackpinion motor Stop_and_rest complete",null);
            }
            v_motor_rackpinion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            counter = 0;
            while (counter < 10 && v_motor_rackpinion.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                counter++;
                sleep(10);
                debugLogException("init", "waiting on rackpinion motor RUN_TO_POSITION complete",null);
            }

        }
        catch (Exception p_exeception)
        {
            debugLogException(config_motor_rackpinion,"missing",p_exeception);
            v_motor_rackpinion = null;
        }

        try
        {
            v_motor_lifter = opMode.hardwareMap.dcMotor.get (config_motor_lifter);
            v_motor_lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //  v_motor_winch.setDirection(DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            debugLogException(config_motor_lifter,"missing",p_exeception);
            v_motor_lifter = null;
        }
        /*try
        {
            // get a reference to our ColorSensor object.
            v_sensor_colorLegecy = hardwareMap.colorSensor.get(v_sensor_colorLegecy_name);
            // bEnabled represents the state of the LED.
            boolean v_sensor_colorLegecy_led_enabled = true;
            // turn the LED on in the beginning, just so user will know that the sensor is active.
            v_sensor_colorLegecy.enableLed(false);

        }
        catch (Exception p_exeception)
        {
            debugLogException(v_sensor_colorLegecy_name, "missing", p_exeception);
            v_sensor_colorLegecy = null;
        }*/

       /* try
        {
            v_sensor_odsLegecy = hardwareMap.opticalDistanceSensor.get ("ods1");

        }
        catch (Exception p_exeception)
        {
            debugLogException("ods1", "missing", p_exeception);
            v_sensor_odsLegecy = null;

        }*/
       /* try
        {
            v_sensor_lightLegecy = hardwareMap.lightSensor.get (v_sensor_lightLegecy_name);

        }
        catch (Exception p_exeception)
        {
            debugLogException(v_sensor_lightLegecy_name, "missing", p_exeception);
            v_sensor_lightLegecy = null;

        }*/

        /*try
        {
            v_sensor_ultraLegecy = hardwareMap.ultrasonicSensor.get (v_sensor_ultraLegecy_name);

        }
        catch (Exception p_exeception)
        {
            debugLogException(v_sensor_ultraLegecy_name, "missing", p_exeception);
            v_sensor_ultraLegecy = null;

        }*/

        try{

            v_ledseg = new AdafruitLEDBackpack7Seg(opMode.hardwareMap, config_i2c_led7seg);

        }catch (Exception p_exeception)
        {
            debugLogException(config_i2c_led7seg, "missing", p_exeception);
            v_ledseg = null;

        }

       /* try{
            if (v_neopixels_use_i2c) {
                v_neopixels = new ArduinoI2CNeopixels(opMode.hardwareMap, "neopixels");
                debugLogException("neopixels", "inited", null);
            }else {
                v_dim.setDigitalChannelMode(v_neopixel_modechange_pin, DigitalChannelController.Mode.OUTPUT);
                v_dim.setDigitalChannelMode(v_neopixel_red_pin, DigitalChannelController.Mode.OUTPUT);
                v_dim.setDigitalChannelMode(v_neopixel_green_pin, DigitalChannelController.Mode.OUTPUT);
                v_dim.setDigitalChannelMode(v_neopixel_blue_pin, DigitalChannelController.Mode.OUTPUT);
                v_dim.setDigitalChannelState(v_neopixel_modechange_pin, v_neopixel_modechange_pin_state);
                v_dim.setDigitalChannelState(v_neopixel_red_pin, true);
                v_dim.setDigitalChannelState(v_neopixel_green_pin, true);
                v_dim.setDigitalChannelState(v_neopixel_blue_pin, false);
            }
        }catch (Exception p_exeception)
        {
            debugLogException("neopixels", "missing", p_exeception);
            v_neopixels = null;
        }*/


        setup_am20();  //chad comment this line for am40 motors

        //update our telmentry after init so we know if we are missing anything

        update_telemetry();
        opMode.telemetry.update();

    } // init

    //--------------------------------------------------------------------------
    //
    // a_warning_generated
    //
    /**
     * Access whether a warning has been generated.
     */
    boolean a_warning_generated ()

    {
        return v_warning_generated;

    } // a_warning_generated

    public boolean play_jingle_bells(){
        //Connect the Core Interface Device or Dim
        if (v_audio_effects != null) {
            v_audio_effects.play_jingle_bells();
            return true;
        }else{
            return false;
        }
    }



    /**
     * Used to retrive the total loop count
     * @return The number of time loop has been executed
     */
    public long loopCounter(){
        return v_loop_ticks;
    }

    void debugLogException(String type, String msg, Exception ex){
        if (ex != null){
            m_warning_message(type);
        }
        String debugMessage = type + ":" + msg;
        if (ex != null) {
            String errMsg = ex.getLocalizedMessage();
            if (errMsg != null) {
                debugMessage = debugMessage + errMsg;
            }
        }
        if (v_debug) {
            //DbgLog.msg(debugMessage);
        }
    }

    private ElapsedTime v_timewait2_elapsedtime;
    private boolean v_is_timewaiting2_complete;
    private int v_timewait2_milliseconds;

    private ElapsedTime v_timewait_elapsedtime;
    private boolean v_is_timewaiting_complete;
    private float v_timewait_seconds;

    public boolean timewait(float seconds){
        v_timewait_elapsedtime = new ElapsedTime();
        v_timewait_seconds = seconds;
        v_is_timewaiting_complete = false;
        return true;
    }

    public boolean timewait_Complete(){
        if ( v_timewait_elapsedtime.seconds() > v_timewait_seconds ){
            v_is_timewaiting_complete = true;
        }
        return v_is_timewaiting_complete;
    }

    private boolean timewait2Milliseconds(int milliseconds){
        v_timewait2_elapsedtime = new ElapsedTime();
        v_timewait2_milliseconds = milliseconds;
        v_is_timewaiting2_complete = false;
        return true;
    }

    private boolean timewait2Milliseconds_Complete(){
        if ( v_timewait2_elapsedtime.milliseconds() >= v_timewait2_milliseconds ){
            v_is_timewaiting2_complete = true;
        }
        return v_is_timewaiting2_complete;
    }

    //--------------------------------------------------------------------------
    //
    // a_warning_message
    //
    /**
     * Access the warning message.
     */
    String a_warning_message ()

    {
        return v_warning_message;

    } // a_warning_message

    //--------------------------------------------------------------------------
    //
    // m_warning_message
    //
    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     *
     * A comma will be added before the specified message if the message isn't
     * empty.
     */
    void m_warning_message (String p_exception_message)

    {
        if (v_warning_generated)
        {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    } // m_warning_message

    //--------------------------------------------------------------------------
    //
    // start
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */



    //--------------------------------------------------------------------------
    //
    // stop
    //
    /**
     * Perform any actions that are necessary when the OpMode is disabled.
     *
     * The system calls this member once when the OpMode is disabled.
     */
    public void stop ()
    {
        //
        // Nothing needs to be done for this method.
        //
        hardware_stop();
    } // stop



    public boolean is_slow_tick(){
        return  v_loop_ticks_slow;
    }


    /** called each time through the loop needed to sync hardware and look for status changes
     *
     */
    private Calendar v_loop_previous_timestamp;
    private long v_slow_loop_milliseconds = 0;
    public void hardware_loop() throws InterruptedException{

        v_loop_ticks++;
        if(v_loop_ticks % v_loop_ticks_slow_count == 0 ){
            v_loop_ticks_slow = true;
            if (v_loop_previous_timestamp != null){
                v_slow_loop_milliseconds = Calendar.getInstance().getTimeInMillis() - v_loop_previous_timestamp.getTimeInMillis();
            }
            v_loop_previous_timestamp = Calendar.getInstance();

        }else{
            v_loop_ticks_slow = false;
        }
        //heartbeat_tick();
        if(v_ledseg != null){
           v_ledseg.loop();
        }
        //if vuforia is inited then we need to update our positions etc

        if(v_loop_ticks_slow){
            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
//            if(v_sensor_gyro != null) {
//                v_sensor_gyro_heading = v_sensor_gyro.getHeading();
//                v_sensor_gyro_x = v_sensor_gyro.rawX();
//                v_sensor_gyro_y = v_sensor_gyro.rawY();
//                v_sensor_gyro_z = v_sensor_gyro.rawZ();
//            }
            //the i2c color sensor uses a memory lock that is taxing so we only do this if we are using the color sensor and ever slow loop count
            if(v_sensor_color_i2c_enabled == true){
                v_sensor_color_i2c_rgbaValues[0] = v_sensor_color_i2c.red();
                v_sensor_color_i2c_rgbaValues[1] = v_sensor_color_i2c.green();
                v_sensor_color_i2c_rgbaValues[2] = v_sensor_color_i2c.blue();
                v_sensor_color_i2c_rgbaValues[3] = v_sensor_color_i2c.alpha();
            }
            vuforia_hardwareLoop();
            if(v_debug) {
                update_telemetry();
                opMode.updateTelemetry(opMode.telemetry);
            }
        }
        opMode.idle();
        waitForTick(5);
    }

    public long hardware_loop_slowtime_milliseconds(){
        return v_slow_loop_milliseconds;
    }

    public void hardware_stop(){
        /*if(v_led_heartbeat !=null){
            v_led_heartbeat.enable(false);
        }*/
        if(v_ledseg != null){
            v_ledseg.stop();
        }

        if(v_motor_left_drive != null){
            v_motor_left_drive.setPower(0);
        }
        if(v_motor_right_drive != null){
            v_motor_right_drive.setPower(0);
        }
        if(v_motor_lifter != null){
            v_motor_lifter.setPower(0);
        }
        if(v_motor_rackpinion != null){
            v_motor_rackpinion.setPower(0);
        }
    }

    //--------------------------------------------------------------------------
    //
    // scale_motor_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    float scale_motor_power (float p_power)
    {
        //
        // Assume no scaling.
        //
        float l_scale = 0.0f;

        //
        // Ensure the values are legal.
        //
        float l_power = Range.clip (p_power, -1, 1);

        float[] l_array =
                { 0.00f, 0.05f, 0.09f, 0.10f, 0.12f
                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
                        , 1.00f, 1.00f
                };

        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int)(l_power * 16.0);
        if (l_index < 0)
        {
            l_index = -l_index;
        }
        else if (l_index > 16)
        {
            l_index = 16;
        }

        if (l_power < 0)
        {
            l_scale = -l_array[l_index];
        }
        else
        {
            l_scale = l_array[l_index];
        }

        return l_scale;

    } // scale_motor_power

    //--------------------------------------------------------------------------
    //
    // a_left_drive_power
    //
    /**
     * Access the left drive motor's power level.
     */
    double a_left_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_left_drive != null)
        {
            l_return = v_motor_left_drive.getPower ();
        }

        return l_return;

    } // a_left_drive_power

    //--------------------------------------------------------------------------
    //
    // a_right_drive_power
    //
    /**
     * Access the right drive motor's power level.
     */
    double a_right_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_right_drive != null)
        {
            l_return = v_motor_right_drive.getPower ();
        }

        return l_return;

    } // a_right_drive_power

    //--------------------------------------------------------------------------
    //
    // set_drive_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    // float l_left_drive_power = 0.0f;
    // float l_right_drive_power = 0.0f;
    public void set_drive_power (float p_left_power, float p_right_power)
    {
        float l_left_drive_power = Range.clip (p_left_power, -1, 1);
        float l_right_drive_power = Range.clip (p_right_power, -1, 1);

        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setPower (l_left_drive_power);
        }
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setPower(l_right_drive_power);
        }
        set_second_message("set_drive_power l" + p_left_power + ":r" + p_right_power + " cliped:l:" + l_left_drive_power +":r" + l_right_drive_power);
    } // set_drive_power

    //--------------------------------------------------------------------------
    //
    // set_drive_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    // float l_left_drive_power = 0.0f;
    // float l_right_drive_power = 0.0f;
    public void set_drive_power_scaled (float p_left_power, float p_right_power)
    {
        float l_left_drive_power = scale_motor_power(p_left_power);
        float l_right_drive_power = scale_motor_power(p_right_power);

        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setPower (l_left_drive_power);
        }
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setPower(l_right_drive_power);
        }
        //set_second_message("set_drive_power " + p_left_power + ":" + p_right_power + " " + l_left_drive_power +":" + l_right_drive_power);
    } // set_drive_power





    public void rackpinion_extend () throws InterruptedException
    {
        if (v_motor_rackpinion != null)
        {
            if (v_rackpinion_isExtended == true){
                set_second_message("rackpinion already extended");
                return;
            }
            v_rackpinion_state = 0;
            v_motor_rackpinion_Position = v_motor_rackpinion_Position + v_motor_rackpinion_ExtendTicks - v_motor_rackpinion_ExtendSlowdownTicks;

            v_motor_rackpinion.setTargetPosition(v_motor_rackpinion_Position);
            set_second_message("extendinging rackpinion");
            v_motor_rackpinion.setPower(v_motor_rackpinion_Speed);

        }
    }

    private int v_rackpinion_state = 0;
    public boolean rackpinion_extend_complete () {
        if (v_motor_rackpinion != null) {
            switch (v_rackpinion_state) {
                case 0:
                    if (v_motor_rackpinion.isBusy() == false) {
                        v_motor_rackpinion_Position = v_motor_rackpinion_Position + v_motor_rackpinion_ExtendSlowdownTicks;
                        v_motor_rackpinion.setPower(0.0F);
                        v_motor_rackpinion.setTargetPosition(v_motor_rackpinion_Position);
                        set_second_message("rackpinion almost extended");
                        v_motor_rackpinion.setPower(0.5F);

                        v_rackpinion_state++;
                    }
                    break;
                case 1:
                    if (v_motor_rackpinion.isBusy() == false) {
                        v_motor_rackpinion.setPower(0.0F);
                        v_rackpinion_isExtended = true;
                        set_second_message("rackpinion loaded");
                        return true;
                    }
                    break;
            }
            return false;

        }else{
            return true;
        }
    }


    //Retract the rackpinion

    //--------------------------------------------------------------------------
    //
    // Retract_rackpinion
    //
    /**
     * Retract the Rack and Pinion
     */
    private boolean v_rackpinion_isExtended = false;
    public void rackpinion_retract ()
    {

        if (v_motor_rackpinion != null )
        {
            if (v_rackpinion_isExtended == false){
                set_second_message("rackpinion not loaded");
                return;
            }
            v_motor_rackpinion_Position = 0;
            v_motor_rackpinion.setTargetPosition(v_motor_rackpinion_Position);
            set_second_message("Retracting rackpinion");
            v_motor_rackpinion.setPower(v_motor_rackpinion_Speed);
        }

    }
    public boolean rackpinion_retract_complete ()
    {
        if (v_motor_rackpinion != null )
        {
            if (v_rackpinion_isExtended == false){
                set_second_message("rackpinion not Extended");
                return true;
            }
            if (v_motor_rackpinion.isBusy()== false) {
                v_motor_rackpinion.setPower(0.0F);
                v_rackpinion_isExtended = false;
                set_second_message("Retracted rackpinion");
                return true;
            }
        }
        return false;
    }

    //--------------------------------------------------------------------------
    //
    // lifter_on
    //
    /**
     * Turn on the lifter motor
     */
    boolean v_motor_lifter_is_on = false;
    public void lifter_on ()
    {

        if (v_motor_lifter != null)
        {
            v_motor_lifter.setPower(v_motor_lifter_power);
            v_motor_lifter_is_on = true;
        }
        set_second_message("lifter_on" + v_motor_lifter_power);
    }

    //--------------------------------------------------------------------------
    //
    // lifter_off
    //
    /**
     * Turn off the lifter motor
     */
    public void lifter_toggle ()
    {
        if (v_motor_lifter_is_on == false) {
            lifter_on();
        }else {
            lifter_off();
        }
    }

    public void lifter_off ()
    {
        if (v_motor_lifter != null)
        {
            v_motor_lifter.setPower(0);
            v_motor_lifter_is_on = false;
        }
        set_second_message("lifter_off" );

    }

    public void lifter_on_reverse ()
    {

        if (v_motor_lifter != null)
        {
            v_motor_lifter.setPower(0-v_motor_lifter_power);
            v_motor_lifter_is_on = true;
        }
        set_second_message("lifter_on" + (0-v_motor_lifter_power));
    }



    //--------------------------------------------------------------------------
    //
    // lifter_off
    //
    /**
     * Turn off the lifter motor
     */
    public void lifter_toggle_reverse ()
    {
        if (v_motor_lifter_is_on == false) {
            lifter_on_reverse();
        }else {
            lifter_off();
        }
    }


    public void run_to_position(float power, float inches ) throws InterruptedException{
        //setupDriveToPosition();
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = v_motor_left_drive.getCurrentPosition() + (int) (inches * v_drive_inches_ticksPerInch);
            newRightTarget = v_motor_right_drive.getCurrentPosition() + (int) (inches * v_drive_inches_ticksPerInch);
            v_motor_left_drive.setTargetPosition(newLeftTarget);
            v_motor_right_drive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            set_drive_power(power, power);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (v_motor_left_drive.isBusy() && v_motor_right_drive.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        v_motor_left_drive.getCurrentPosition(),
                        v_motor_right_drive.getCurrentPosition());
                opMode.telemetry.update();
            }
        }
    }

    public void run_without_encoders(){
        v_motor_left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        v_motor_right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //--------------------------------------------------------------------------
    //
    // run_using_left_drive_encoder
    //
    /**
     * Set the left drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_left_drive_encoder ()

    {
        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setMode
                    (DcMotor.RunMode.RUN_USING_ENCODER
                    );
        }

    } // run_using_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_right_drive_encoder
    //
    /**
     * Set the right drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_right_drive_encoder ()

    {
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setMode
                    (DcMotor.RunMode.RUN_USING_ENCODER
                    );
        }

    } // run_using_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_encoders
    //
    /**
     * Set both drive wheel encoders to run, if the mode is appropriate.
     */
    public void run_using_encoders ()

    {
        //
        // Call other members to perform the action on both motors.
        //
        run_using_left_drive_encoder ();
        run_using_right_drive_encoder ();

    } // run_using_encoders
/*

    //--------------------------------------------------------------------------
    //
    // run_without_left_drive_encoder
    //
    */
    /**
     * Set the left drive wheel encoder to run, if the mode is appropriate.
     *//*

    public void run_without_left_drive_encoder ()

    {
        if (v_motor_left_drive != null)
        {
            if (v_motor_left_drive.getMode() ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_left_drive.setMode
                        (DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_left_drive_encoder
*/

    public boolean sound_play_dtmf(int tone, int duration){
        if (v_tone_generator != null) {
            v_tone_generator.startTone(tone, duration);
            return true;
        }else{
            return false;
        }
    }

    /*//--------------------------------------------------------------------------
    //
    // run_without_right_drive_encoder
    //
    *//**
     * Set the right drive wheel encoder to run, if the mode is appropriate.
     *//*
    public void run_without_right_drive_encoder ()

    {
        if (v_motor_right_drive != null)
        {
            if (v_motor_right_drive.getMode() ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_right_drive.setMode
                        (DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_without_drive_encoders
    //
    *//**
     * Set both drive wheel encoders to run, if the mode is appropriate.
     *//*
    public void run_without_drive_encoders ()

    {
        //
        // Call other members to perform the action on both motors.
        //
        run_without_left_drive_encoder ();
        run_without_right_drive_encoder ();

    } // run_without_drive_encoders
*/
    //--------------------------------------------------------------------------
    //
    // reset_left_drive_encoder
    //
    /**
     * Reset the left drive wheel encoder.
     */
    public void reset_left_drive_encoder ()

    {
        if (v_motor_left_drive != null)
        {
            //This may Cause a Stop Now versus just a rest andy 09/24/2016
            v_motor_left_drive.setMode
                    (DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    );
        }

    } // reset_left_drive_encoder

    public boolean isInDriveMode(DcMotor.RunMode RunMode){
        if (v_motor_left_drive != null && v_motor_right_drive != null){
            if(v_motor_left_drive.getMode() == RunMode && v_motor_right_drive.getMode() == RunMode){
                return true;
            }else{
                return false;
            }
        }
        return true;
    }

    //--------------------------------------------------------------------------
    //
    // reset_right_drive_encoder
    //
    /**
     * Reset the right drive wheel encoder.
     */
    public void reset_right_drive_encoder ()

    {
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setMode
                    (DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    );
        }

    } // reset_right_drive_encoder

    public void setupAutoDrive() {

        if(isInDriveMode(DcMotor.RunMode.RUN_USING_ENCODER) == false){
            set_drive_power(0.0f,0.0f);
            run_using_encoders();
            int counter = 0;

            while (counter < 10 &&isInDriveMode(DcMotor.RunMode.RUN_USING_ENCODER)==false){
                counter++;
                sleep(100);
                debugLogException("init", "waiting on  DcMotor.RunMode.RUN_USING_ENCODER) complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
            }

        }

    }
    public void setupManualDrive() {

        if (isInDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) == false){
            set_drive_power(0.0f,0.0f);
            run_using_encoders();
            int counter = 0;
            while (counter < 10 && isInDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)==false){
                counter++;
                sleep(100);
                debugLogException("init", "waiting on  DcMotor.RunMode.RUN_WITHOUT_ENCODER) complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
            }
        }


    }

    public void setupDriveToPosition(){

        if (isInDriveMode(DcMotor.RunMode.RUN_TO_POSITION) == false){
            //reset_drive_encoders();
            int counter = 0;
            /*while (counter < 20 && isInDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)==false){
                counter++;
                sleep(100);
                debugLogException("init", "waiting on  DcMotor.RunMode.STOP_AND_RESET_ENCODER) complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
            }*/
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (counter < 10 && isInDriveMode(DcMotor.RunMode.RUN_TO_POSITION)==false){
                counter++;
                sleep(100);
                debugLogException("init", "waiting on  DcMotor.RunMode.RUN_TO_POSITION) complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
            }
        }

    }
/*
    public void setDriveToPosition(){

        if (isInDriveMode(DcMotor.RunMode.RUN_TO_POSITION) == false){
            //reset_drive_encoders();
            int counter = 0;
            //while (counter < 10 && isInDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)==false){
            //    counter++;
            //    sleep(100);
            //    debugLogException("init", "waiting on  DcMotor.RunMode.STOP_AND_RESET_ENCODER) complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
            //}
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (counter < 10 && isInDriveMode(DcMotor.RunMode.RUN_TO_POSITION)==false){
                counter++;
                sleep(100);
                debugLogException("init", "waiting on  DcMotor.RunMode.RUN_TO_POSITION) complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
            }
        }

    }*/

    //--------------------------------------------------------------------------
    //
    // reset_drive_encoders
    //
    /**
     * Reset both drive wheel encoders.
     */
    public void reset_drive_encoders ()

    {
        //
        // Reset the motor encoders on the drive wheels.
        //
        reset_left_drive_encoder();
        reset_right_drive_encoder();

    } // reset_drive_encoders

    //--------------------------------------------------------------------------
    //
    // a_left_encoder_count
    //
    /**
     * Access the left encoder's count.
     */
    int a_left_encoder_count ()
    {
        int l_return = 0;

        if (v_motor_left_drive != null)
        {
            l_return = v_motor_left_drive.getCurrentPosition ();
        }

        return l_return;

    } // a_left_encoder_count

    /**
     * Access the left drive mode.
     */
    DcMotor.RunMode a_left_drive_mode ()
    {


        if (v_motor_left_drive != null)
        {
            return v_motor_left_drive.getMode();
        }

        return DcMotor.RunMode.RUN_TO_POSITION;

    } // a_left_drive_mode

    /**
     * Access the right drive mode.
     */
    DcMotor.RunMode a_right_drive_mode ()
    {


        if (v_motor_right_drive != null)
        {
            return v_motor_right_drive.getMode();
        }

        return DcMotor.RunMode.RUN_TO_POSITION;

    } // a_right_drive_mode


/*
    public boolean neopixels_set_rgb(byte red, byte green, byte blue){
        //I2c didn't work seemed to bring down whole bus on modern Robotics controller will work on it later
        // so we use digital io for now

        if (v_dim != null){
            if (v_neopixels_use_i2c && v_neopixels != null ) {
                v_neopixels.set_rgb(red, green, blue);
            }else {
                if (red > 0) {
                    v_dim.setDigitalChannelState(v_neopixel_red_pin, false);
                } else {
                    v_dim.setDigitalChannelState(v_neopixel_red_pin, true);
                }
                if (green > 0) {
                    v_dim.setDigitalChannelState(v_neopixel_green_pin, false);
                } else {
                    v_dim.setDigitalChannelState(v_neopixel_green_pin, true);
                }
                if (blue > 0) {
                    v_dim.setDigitalChannelState(v_neopixel_blue_pin, false);
                } else {
                    v_dim.setDigitalChannelState(v_neopixel_blue_pin, true);
                }
            }
            return true;
        }else{
            return false;
        }
    }

    public boolean neopixels_set_brightness(byte brightness){
        if (v_neopixels != null){
            v_neopixels.set_brightness(brightness);
            return true;
            }else{
                return false;
        }
    }

    boolean v_neopixel_modechange_pin_state = false;
    public boolean neopixels_set_mode(byte mode){

        if (v_dim != null){
            if (v_neopixels_use_i2c && v_neopixels != null){
                 v_neopixels.set_mode(mode);
            }else{
                v_neopixel_modechange_pin_state = !v_neopixel_modechange_pin_state;
                v_dim.setDigitalChannelState(v_neopixel_modechange_pin, v_neopixel_modechange_pin_state);
            }
            return true;
        }else{
            return false;
        }
    }
*/

    //--------------------------------------------------------------------------
    //
    // a_right_encoder_count
    //
    /**
     * Access the right encoder's count.
     */
    int a_right_encoder_count ()

    {
        int l_return = 0;

        if (v_motor_right_drive != null)
        {
            l_return = v_motor_right_drive.getCurrentPosition();
        }

        return l_return;

    } // a_right_encoder_count

    //--------------------------------------------------------------------------
    //
    // has_left_drive_encoder_reached
    //
    /**
     * Indicate whether the left drive motor's encoder has reached a value.
     */
    boolean has_left_drive_encoder_reached (double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_left_drive != null)
        {
            //
            // Has the encoder reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (v_motor_left_drive.getCurrentPosition ()) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_left_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // has_right_drive_encoder_reached
    //
    /**
     * Indicate whether the right drive motor's encoder has reached a value.
     */
    boolean has_right_drive_encoder_reached (double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_right_drive != null)
        {
            //
            // Have the encoders reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (v_motor_right_drive.getCurrentPosition ()) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_right_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reached
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean have_drive_encoders_reached
    ( double p_left_count
            , double p_right_count
    )

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached the specified values?
        //
        if (has_left_drive_encoder_reached (p_left_count) &&
                has_right_drive_encoder_reached (p_right_count))
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_encoders_reached

    //--------------------------------------------------------------------------
    //
    // drive_using_encoders
    //

    /**
     * *
     * Indicate whether the drive motors' encoders have reached a value.
     * @param p_left_power  Power 0.0-1.0 for left motor
     * @param p_right_power Power 0.0-1.0 for left motor
     * @param p_left_count  Encoder ticks to travel before stopping
     * @param p_right_count Encoder ticks to travel before stopping
     * @param useGyro   If true then the gyro will try to maintain a heading by slightly decreasing a tracks power
     * @param desiredHeading the desired track call sensor_gyro_heading to get current heading
     * @return true if we have reached the desired distance
     */

    public boolean drive_using_encoders
    ( float p_left_power
            , float p_right_power
            , double p_left_count
            , double p_right_count
            , boolean useGyro
            , int desiredHeading
    )

    {
        //
        // Assume the encoders have not reached the limit.
        //
        boolean l_return = false;

        //
        // Tell the system that motor encoders will be used.
        //
        run_using_encoders ();

        //
        // Start the drive wheel motors at full power.
        //
        set_drive_power (p_left_power, p_right_power);

        //
        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (have_drive_encoders_reached (p_left_count, p_right_count))
        {

            //
            // Stop the motors.
            //
            set_drive_power (0.0f, 0.0f);

            //
            // Transition to the next state when this method is called
            // again.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // drive_using_encoders

    //--------------------------------------------------------------------------
    //
    // has_left_drive_encoder_reset
    //
    /**
     * Indicate whether the left drive encoder has been completely reset.
     */
    boolean has_left_drive_encoder_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the left encoder reached zero?
        //
        if (a_left_encoder_count() == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_left_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // has_right_drive_encoder_reset
    //
    /**
     * Indicate whether the left drive encoder has been completely reset.
     */
    boolean has_right_drive_encoder_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the right encoder reached zero?
        //
        if (a_right_encoder_count() == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_right_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reset
    //
    /**
     * Indicate whether the encoders have been completely reset.
     */
    boolean have_drive_encoders_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached zero?
        //
        if (has_left_drive_encoder_reset() && has_right_drive_encoder_reset ())
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_drive_encoders_reset

    private long v_drive_inches_ticks_target_right_slowdown;
    private long v_drive_inches_ticks_target_left_slowdown;
    private long v_drive_inches_ticks_target_right_stop;
    private long v_drive_inches_ticks_target_left_stop;
    private int v_drive_inches_ticks_target_right;
    private int v_drive_inches_ticks_target_left;
    private float v_drive_inches_power;
    private boolean v_drive_inches_useGyro;
    private int v_drive_inches_state;
    private int v_drive_inches_heading;
    private boolean v_drive_slowdown1_already_set = false;


    // inches is positive
    public void drive_inches(float inches, boolean useGyro){
        if(inches < 0){
            //added a reverse power as we are rear heavey need to run slower backward so not to tip over
            drive_inches(v_drive_power_reverse, inches, useGyro);
        }else {
            drive_inches(v_drive_power, inches, useGyro);
        }
    }

    private void drive_inches(float power,float inches, boolean useGyro){
        try {

            if (v_motor_left_drive == null || v_motor_right_drive == null){
                return;
            }
            if (power < 0 ){
                set_second_message("Power should be positive");
                return;
            }
            setupDriveToPosition();
            v_drive_inches_power = power;

            int v_left_position =  a_left_encoder_count();
            int v_right_position =  a_right_encoder_count();
            //if (inches >= 0 ){
                //we are going forward
                v_drive_inches_ticks_target_right = v_right_position +  (int)Math.round(inches * v_drive_inches_ticksPerInch);
                v_drive_inches_ticks_target_left = v_left_position +  (int)Math.round(inches * v_drive_inches_ticksPerInch);

            //}else{
                //we are going backward
            //    v_drive_inches_ticks_target_right = v_right_position - (int) Math.round(inches * v_drive_inches_ticksPerInch);
            //    v_drive_inches_ticks_target_left = v_left_position - (int) Math.round(inches * v_drive_inches_ticksPerInch);
            //    v_drive_inches_power = 0 -v_drive_inches_power;  //run in reverse
            //}
            v_drive_inches_useGyro = useGyro;
            v_drive_inches_state = 0;

            v_motor_left_drive.setTargetPosition(v_drive_inches_ticks_target_left);
            v_motor_right_drive.setTargetPosition(v_drive_inches_ticks_target_right);
            set_second_message("drive_inches: p: " + v_drive_inches_power
                    + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
                    + ",l:" + v_left_position + ",r:" + v_right_position );

        }catch (Exception p_exeception)
        {
            debugLogException("drive inches", "drive_inches", p_exeception);


        }
    }


    public boolean drive_inches_complete(){
        if (v_motor_left_drive == null || v_motor_right_drive == null){
            return true;
        }

        int v_drive_inches_ticks_left = v_motor_left_drive.getCurrentPosition();
        int v_drive_inches_ticks_right = v_motor_right_drive.getCurrentPosition();

        switch(v_drive_inches_state){

            case 0:
                if (v_drive_inches_useGyro) {
                    v_drive_inches_heading = sensor_gyro_get_heading();
                }
                //add a delay to handle any stupid issues with the ftc dc motor controller timming
                //Commented by Andy Add Back if needed 11/10/2016 sleep(2);

                set_drive_power(v_drive_inches_power, v_drive_inches_power);
                set_second_message("drive_inches_complete: set the drive power "
                        +  " p: " + v_drive_inches_power
                        + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
                        + ",l:" + v_drive_inches_ticks_left + ",r:" + v_drive_inches_ticks_right);
                v_drive_inches_state++;
                break;
            case 1:
                if(v_motor_left_drive.isBusy() == false && v_motor_right_drive.isBusy() == false) {
                    return true;
                }else if(v_drive_use_slowdown == true ){
                    int v_drive_left_difference =  v_drive_inches_ticks_target_left - v_drive_inches_ticks_left;
                    if(v_drive_left_difference < 0){
                        v_drive_left_difference = 0 - v_drive_left_difference;
                    }
                    int v_drive_right_difference =  v_drive_inches_ticks_target_right - v_drive_inches_ticks_right;
                    if(v_drive_right_difference < 0){
                        v_drive_right_difference = 0 - v_drive_right_difference;
                    }

                    if(v_drive_left_difference <= v_drive_inches_slowdown_ticks || v_drive_right_difference <= v_drive_inches_slowdown_ticks){
                        set_drive_power(v_drive_power_slowdown, v_drive_power_slowdown);
                    }
                }
                /*else if(v_drive_inches_useGyro){
                    //the logic here is to try to hold a gyro heading by slowing a track down a touch v_drive_inches_power_gyro_correction
                    //the issue is our gyro is slow to refresh so may need to only do this a couple of loops then turn off
                    //not sure yet still testing 11/19/2015 Two days to first competition wow this is tight deadline
                    int headingDifference;
                    int currentHeading = sensor_gyro_get_heading();
                    if ((v_drive_inches_heading - currentHeading) > 180 ) {
                        currentHeading = 360 + currentHeading;
                    }else if ((v_drive_inches_heading - currentHeading) < -180 ) {
                        currentHeading = currentHeading - 360;
                    }
                    headingDifference = Math.abs(currentHeading-v_drive_inches_heading);
                    //hard limit of no more then 3 times the correction
                    if (headingDifference > v_drive_inches_power_gyro_correction_max_times){
                        headingDifference = v_drive_inches_power_gyro_correction_max_times;
                    }
                    float powerCorrectAmount = v_drive_inches_power_gyro_correction * headingDifference;
                    if (v_drive_inches_power < 0.0d){
                        powerCorrectAmount = 0 - powerCorrectAmount;
                    }
                    float v_left_power_adjust = v_drive_inches_power;
                    float v_right_power_adjust = v_drive_inches_power;
                    if ((v_drive_inches_heading > currentHeading && v_drive_inches_power > 0) || (v_drive_inches_heading < currentHeading && v_drive_inches_power < 0) ){
                        v_right_power_adjust = v_drive_inches_power - powerCorrectAmount;
                    }else if ((v_drive_inches_heading < currentHeading && v_drive_inches_power > 0) || (v_drive_inches_heading > currentHeading && v_drive_inches_power < 0)){
                        v_left_power_adjust = v_drive_inches_power - powerCorrectAmount;
                    }
                    set_drive_power(v_left_power_adjust, v_right_power_adjust );

                }*/
                if(v_loop_ticks_slow) {
                    set_second_message("drive_inches_complete: "
                            + "gyro th:" + v_drive_inches_heading
                            +  " p: " + v_drive_inches_power
                            + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
                            + ",l:" + v_drive_inches_ticks_left + ",r:" + v_drive_inches_ticks_right
                            + ",lp:" + v_motor_left_drive.getPower() + ",rp:" + v_motor_right_drive.getPower()
                    );
                }
                break;
            default:
                return true;
        }

        return false;
    }


    /**
     * Inits the led 7 segment counter to start a count down in seconds
     * @param seconds
     * @return
     */
    public boolean led7seg_timer_init(int seconds){
        return false;
    }

    private boolean isFirstButtonPress = true;

    public boolean manualModeButtonPress(){
        if (isFirstButtonPress){
            isFirstButtonPress = false;
            v_ledseg.startTimer(120);
        }
        return true;
    }

    /**
     * Inits the led 7 segment counter to start a count down in seconds
     * @return
     */

    public boolean led7seg_test(){
        if (v_ledseg != null){

            v_ledseg.writetest();
            return true;
        }
        return false;
    }

    /**
     * starts led 7 segment counter to to count down in seconds
     * @param seconds
     * @return
     */
    int v_led7seg_timer_seconds = 0;
    boolean v_led7seg_timer_running = false;
    public boolean led7seg_timer_start(int seconds){
        if (v_ledseg != null){

            v_ledseg.startTimer(seconds);
            return true;
        }
        return false;
    }

    public boolean led7seg_timer_complete(){
        if (v_ledseg != null) {
            return v_ledseg.is_timer_complete();
        }else{
            return true;
        }
    }
    public boolean led7seg_is_enabled(){
        if (v_ledseg != null){
            return v_ledseg.isEnabled();
        }
        return false;
    }
    public boolean led7seg_enabled(boolean enabled){
        if (v_ledseg != null){
            return v_ledseg.enabled(enabled);
        }
        return false;
    }

    private int v_drive_ToPosition_ticks_target_right;
    private int v_drive_ToPosition_ticks_target_left;

    public void drive_ToPosition(int leftTicks, int rightTicks, float leftPower, float rightPower){
        setupDriveToPosition();
        if( v_motor_left_drive == null || v_motor_right_drive == null){
            return;
        }
        int v_motor_right_position = v_motor_right_drive.getCurrentPosition();
        int v_motor_left_position = v_motor_left_drive.getCurrentPosition();
        v_drive_ToPosition_ticks_target_left = v_motor_left_position + leftTicks;
        v_drive_ToPosition_ticks_target_right = v_motor_right_position + rightTicks;

        v_motor_right_drive.setTargetPosition(v_drive_ToPosition_ticks_target_right);
        v_motor_left_drive.setTargetPosition(v_drive_ToPosition_ticks_target_left);
        set_drive_power(leftPower,rightPower);
        set_second_message("drive_ToPosition: lt:" + v_drive_ToPosition_ticks_target_left + " rt:" + v_drive_ToPosition_ticks_target_right + ", re:" + v_motor_right_position + ", le:" + v_motor_left_position );
    }

    public boolean drive_ToPosition_Complete(){
        if( v_motor_left_drive == null || v_motor_right_drive == null){
            return true;
        }

        if (v_motor_left_drive.isBusy() == false
                &&  v_motor_right_drive.isBusy() == false) {
            set_drive_power(0.0f, 0.0f);

            int v_motor_right_position = v_motor_right_drive.getCurrentPosition();
            int v_motor_left_position = v_motor_left_drive.getCurrentPosition();
            set_second_message("drive_ToPosition_Complete: lt:" + v_drive_ToPosition_ticks_target_left + " rt:" + v_drive_ToPosition_ticks_target_right + ", re:" + v_motor_right_position + ", le:" + v_motor_left_position);
            return true;
        }else{
            if(is_slow_tick()){
                int v_motor_right_position = v_motor_right_drive.getCurrentPosition();
                int v_motor_left_position = v_motor_left_drive.getCurrentPosition();
                set_second_message("drive_ToPosition_Complete: lt:" + v_drive_ToPosition_ticks_target_left + " rt:" + v_drive_ToPosition_ticks_target_right + ", re:" + v_motor_right_position + ", le:" + v_motor_left_position);
            }
            return false;
        }

    }

    private int v_turn_degrees_ticks_target_right;
    private int v_turn_degrees_ticks_target_left;

    private int v_turn_degrees_heading_target;
    private int v_turn_degrees_heading_target_slow;
    private int v_turn_degrees_heading_target_stop;
    private boolean v_turn_degrees_heading_target_360round;
    private boolean v_turn_degrees_heading_start_error_360round;
    private boolean v_turn_degrees_heading_target_slow_360round;
    private boolean v_turn_degrees_heading_target_stop_360round;
    private int v_turn_degrees_heading_start;
    private static final int v_turn_degrees_heading_overshoot_slowdown = 30;
    private static final int v_turn_degrees_heading_overshoot_stop = 5;
    private int v_turn_degrees_heading_start_error;
    private boolean v_turn_degrees_usingGyro;
    private boolean v_turn_degrees_iscwturn;
    private boolean v_turn_degrees_isSlowTurn;
    private int v_turn_degrees_state;
    /**
     *
     * @param degrees the amount in degrees you want to turn postive number is to the right negitive to the left
     * @param turnSlow make a slowTurn
     * @param useGyro use the Gyro to turn if false then ticks of the encoder will be used
     *
     */



    public void turn_degrees(int degrees, boolean turnSlow, boolean useGyro){
        //Do nothing is turn is zero
        if(degrees == 0 || v_motor_left_drive == null || v_motor_right_drive == null){
            return;
        }

        setupDriveToPosition();
        v_turn_degrees_state = 0;
        v_turn_degrees_usingGyro = useGyro;
        v_turn_degrees_isSlowTurn = turnSlow;
        if (degrees > 0) {
            //greater then 0 turn cw or to the right
            v_turn_degrees_iscwturn = true;
        } else {
            v_turn_degrees_iscwturn = false;
        }


            //Turn using just encoder ticks
        int ticks = Math.round(Math.abs(degrees) * v_drive_turn_ticks_per_degree);
        if (v_turn_degrees_iscwturn) {
            v_turn_degrees_ticks_target_left = v_motor_left_drive.getCurrentPosition() + ticks;
            v_turn_degrees_ticks_target_right = v_motor_right_drive.getCurrentPosition() - ticks;
        } else {
            v_turn_degrees_ticks_target_left = v_motor_left_drive.getCurrentPosition() - ticks;
            v_turn_degrees_ticks_target_right =  v_motor_right_drive.getCurrentPosition() + ticks;
        }
        v_motor_right_drive.setTargetPosition(v_turn_degrees_ticks_target_right);
        v_motor_left_drive.setTargetPosition(v_turn_degrees_ticks_target_left);
        set_second_message("turn_degrees: ticks:" + ticks + ", lt:" + v_turn_degrees_ticks_target_left + " rt:" + v_turn_degrees_ticks_target_right + ", re:" + v_motor_right_drive.getCurrentPosition() + ", le:" + v_motor_left_drive.getCurrentPosition() );
    }

    /**
     * Used to tell if the turn is complete turn_degrees must be called first
     *
     * @return true if the turn is complete false if not
     */

    public boolean turn_complete(){

        try{

            if(v_motor_left_drive == null || v_motor_right_drive == null){
                set_second_message("turn_complete: no motors");
                v_turn_degrees_state = -1;
                return true;
            }

            switch(v_turn_degrees_state){
                case 0:
                    if (v_turn_degrees_iscwturn) {
                        if (v_turn_degrees_isSlowTurn) {
                            //have an issue where motors not turning on at same time so need to call directly
                            //turning right so turn on left motor first
                            v_motor_left_drive.setPower(v_turn_motorspeed_slow);
                            v_motor_right_drive.setPower(0 - v_turn_motorspeed_slow);
                            //set_drive_power(v_turn_motorspeed_slow, 0-v_turn_motorspeed_slow);
                            //sleep(2);
                            set_second_message("turn_complete: set slow turn cw r:" + v_motor_right_drive.getPower() + "l:" + v_motor_left_drive.getPower());


                        }else {
                            //turning right so turn on left motor first
                            v_motor_left_drive.setPower(v_turn_motorspeed);
                            v_motor_right_drive.setPower(0 - v_turn_motorspeed);
                            //set_drive_power(v_turn_motorspeed, 0- v_turn_motorspeed);
                            //sleep(2);
                            set_second_message("turn_complete: set fast turn cw r:" + v_motor_right_drive.getPower() + "l:" + v_motor_left_drive.getPower());

                        }
                        //v_motor_left_drive.setPowerFloat();
                        //v_motor_right_drive.setPowerFloat();
                    } else {
                        if (v_turn_degrees_isSlowTurn) {
                            //turning left so turn on right motor first
                            v_motor_right_drive.setPower(v_turn_motorspeed_slow);
                            v_motor_left_drive.setPower(0 - v_turn_motorspeed_slow);
                            //set_drive_power(0-v_turn_motorspeed_slow,  v_turn_motorspeed_slow);
                            //sleep(2);
                            set_second_message("turn_complete: set slow turn ccw r:" + v_motor_right_drive.getPower() + "l:" + v_motor_left_drive.getPower());
                        }else {
                            //turning left so turn on right motor first
                            v_motor_right_drive.setPower(v_turn_motorspeed);
                            v_motor_left_drive.setPower(0 - v_turn_motorspeed);
                            set_drive_power(0-v_turn_motorspeed, v_turn_motorspeed);
                            //sleep(2);
                            set_second_message("turn_complete: set fast turn ccw r:" + v_motor_right_drive.getPower() + "l:" + v_motor_left_drive.getPower() );
                        }
                        //v_motor_right_drive.setPowerFloat();
                        //v_motor_left_drive.setPowerFloat();
                    }

                    v_turn_degrees_state++;
                    break;
                case 1:
                        if (v_motor_left_drive.isBusy() == false
                                &&  v_motor_right_drive.isBusy() == false) {
                            set_drive_power(0.0f, 0.0f);
                            set_second_message("turn_complete: encoders reached value lt:" + v_turn_degrees_ticks_target_left + " rt:" + v_turn_degrees_ticks_target_right + ", re:" + v_motor_right_drive.getCurrentPosition() + ", le:" + v_motor_left_drive.getCurrentPosition() + ", rp:" + v_motor_right_drive.getPower() + ", lp:" + v_motor_left_drive.getPower());
                            v_turn_degrees_state++;
                            return true;
                        }else {
                            if (is_slow_tick()){
                                set_second_message("turn_complete: Waiting on encoders lt:" + v_turn_degrees_ticks_target_left + " rt:" + v_turn_degrees_ticks_target_right + ", re:" + v_motor_right_drive.getCurrentPosition() + ", le:" + v_motor_left_drive.getCurrentPosition() + ", rp:" + v_motor_right_drive.getPower() + ", lp:" + v_motor_left_drive.getPower() );
                            }
                        }

                    break;
                default:
                    return true;
            }

            return false;
        } catch (Exception p_exeception)
        {
            debugLogException("turn_complete:", "error " + p_exeception.getMessage() , p_exeception);
            return false;

        }
    }




//    //lifter On
//    boolean lifter_On ()
//    {
//        m_winch_power(v_motor_winch_Speed);
//        return true;
//
//    } // rpaarm_moveUp
//    //--------------------------------------------------------------------------
//    //
//    // m_winch_power
//    //
//    /**
//     * Access the winch motor's power level.
//     */
//    void m_lifter_power (double p_level)
//    {
//        if (v_motor_winch != null)
//        {
//            if(p_level > 0){
//                //move the rpa base arm down at same time so not to fight the winch with the servo
//                rpabase_moveDown(true);
//                v_motor_winch.setPower(p_level);
//            }
//            else{
//                v_motor_winch.setPower(0);
//            }
//        }
//
//    } // m_winch_power
//



    boolean v_server_pushbutton_left_is_extended = false;

    //--------------------------------------------------------------------------
    //
    // pushbutton_left_toggle
    //
    /**
     * toggle the left push button servo between extended and retracted positions.
     */
    public void pushbutton_left_toggle ()
    {
        try {
            if (v_server_pushbutton_left_is_extended == true){
                pushbutton_left_retract();
            }else{
                pushbutton_left_extend();
            }
        }catch (Exception p_exeception)
        {
            debugLogException("pushbutton_left_toggle", "error", p_exeception);
        }
    }


    //--------------------------------------------------------------------------
    //
    // pushbutton_left_extend
    //
    /**
     * move the left push button servo to its full extention position.
     */
    public void pushbutton_left_extend ()
    {
        try {
            if (v_servo_pushbutton_left != null) {
                v_servo_pushbutton_left.setPosition(v_servo_pushbutton_left_MinPosition);
                v_server_pushbutton_left_is_extended = true;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("pushbutton_left_extend", "error", p_exeception);
        }
    }


    //--------------------------------------------------------------------------
    //
    // pushbutton_left_extend
    //
    /**
     * move the left push button servo to its retracted position.
     */
    public void pushbutton_left_retract ()
    {
        try {
            if (v_servo_pushbutton_left != null) {
                v_server_pushbutton_left_is_extended = false;
                v_servo_pushbutton_left.setPosition(v_servo_pushbutton_left_MaxPosition);
            }
        }catch (Exception p_exeception)
        {
            debugLogException("pushbutton_left_retract", "error", p_exeception);
        }
    }

    boolean v_server_pushbutton_right_is_extended = false;
    //--------------------------------------------------------------------------
    //
    // pushbutton_right_extend
    //
    /**
     * move the right push button servo to its full extention position.
     */
    public void pushbutton_right_extend ()
    {
        try {
            if (v_servo_pushbutton_right != null) {
                v_servo_pushbutton_right.setPosition(v_servo_pushbutton_right_MaxPosition);
                v_server_pushbutton_right_is_extended = true;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("pushbutton_right_extend", "error", p_exeception);
        }
    }

    public void blockgrabber_toggle ()
    {
        try {
            if (v_servo_blockgrabber_is_extended == true){
                blockgrabber_retract();
            }else{
                blockgrabber_extend();
            }
        }catch (Exception p_exeception)
        {
            debugLogException("blockgrabber_toggle", "error", p_exeception);
        }
    }
    public void blockgrabber_extend ()
    {
        try {
            if (v_servo_blockgrabber != null) {
                v_servo_blockgrabber.setPosition(v_servo_blockgrabber_MinPosition);
                v_servo_blockgrabber_is_extended = true;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("blockgrabber_extend", "error", p_exeception);
        }
    }

    public void blockgrabber_retract ()
    {
        try {
            if (v_servo_blockgrabber != null) {
                v_servo_blockgrabber_is_extended = false;
                v_servo_blockgrabber.setPosition(v_servo_blockgrabber_MaxPosition);
            }
        }catch (Exception p_exeception)
        {
            debugLogException("blockgrabber_retract", "error", p_exeception);
        }
    }

    public void blockgrabber_middle ()
    {
        try {
            if (v_servo_blockgrabber != null) {
                v_servo_blockgrabber_is_extended = false;
                v_servo_blockgrabber.setPosition(v_servo_blockgrabber_MiddlePosition);
            }
        }catch (Exception p_exeception)
        {
            debugLogException("blockgrabber_retract", "error", p_exeception);
        }
    }


    public void elbow_toggle ()
    {
        try {
            if (v_servo_elbow_is_extended == true){
                elbow_retract();
            }else{
                elbow_extend();
            }
        }catch (Exception p_exeception)
        {
            debugLogException("elbow_toggle", "error", p_exeception);
        }
    }
    public void elbow_extend ()
    {
        try {
            if (v_servo_elbow != null) {
                v_servo_elbow.setPosition(v_servo_elbow_MinPosition);
                v_servo_elbow_is_extended = true;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("elbow_extend", "error", p_exeception);
        }
    }

    public void elbow_retract ()
    {
        try {
            if (v_servo_elbow != null) {
                v_servo_elbow_is_extended = false;
                v_servo_elbow.setPosition(v_servo_elbow_MaxPosition);
            }
        }catch (Exception p_exeception)
        {
            debugLogException("elbow_retract", "error", p_exeception);
        }
    }



    public void shoulder_toggle ()
    {
        try {
            if (v_servo_shoulder_is_extended == true){
                shoulder_retract();
            }else{
                shoulder_extend();
            }
        }catch (Exception p_exeception)
        {
            debugLogException("shoulder_toggle", "error", p_exeception);
        }
    }
    public void shoulder_extend ()
    {
        try {
            if (v_servo_shoulder != null) {
                v_servo_shoulder.setPosition(v_servo_shoulder_MinPosition);
                v_servo_shoulder_is_extended = true;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("shoulder_extend", "error", p_exeception);
        }
    }

    public void shoulder_retract ()
    {
        try {
            if (v_servo_shoulder != null) {
                v_servo_shoulder_is_extended = false;
                v_servo_shoulder.setPosition(v_servo_shoulder_MaxPosition);
            }
        }catch (Exception p_exeception)
        {
            debugLogException("shoulder_retract", "error", p_exeception);
        }
    }

    public void debugOff(){
        v_debug = false;
        set_second_message("Debug is Off no Telemetry Enabled");
        update_telemetry();
        opMode.updateTelemetry(opMode.telemetry);
    }
    //--------------------------------------------------------------------------
    //
    // pushbutton_right_retract
    //
    /**
     * move the left push button servo to its retracted position.
     */
    public void pushbutton_right_retract ()
    {
        try {
            if (v_servo_pushbutton_right != null) {
                v_server_pushbutton_right_is_extended = false;
                v_servo_pushbutton_right.setPosition(v_servo_pushbutton_right_MinPosition);
            }
        }catch (Exception p_exeception)
        {
            debugLogException("pushbutton_right_retract", "error", p_exeception);
        }
    }

    //--------------------------------------------------------------------------
    //
    // pushbutton_right_toggle
    //
    /**
     * toggle the right push button servo between extended and retracted positions.
     */
    public void pushbutton_right_toggle ()
    {
        try {
            if (v_server_pushbutton_right_is_extended == true){
                pushbutton_right_retract();
            }else{
                pushbutton_right_extend();
            }
        }catch (Exception p_exeception)
        {
            debugLogException("pushbutton_right_toggle", "error", p_exeception);
        }
    }

    //--------------------------------------------------------------------------
    //
    // arm_wrist_moveRight
    //
    /**
     * move the arm_wrist servo to the Right.
     */
  /*  double arm_wrist_moveRight (boolean fast)
    {
        double l_temptarget;
        if (fast) {
            l_temptarget = a_arm_wrist_position() - ArmWristServo_Delta_Fast;
        }else{
            l_temptarget = a_arm_wrist_position() - ArmWristServo_Delta;
        }
        return m_arm_wrist_position(l_temptarget);
    } // arm_wrist_moveRight

    //--------------------------------------------------------------------------
    //
    // m_arm_wrist_position
    //
    /**
     * Mutate the arm wrist position.
     */
  /*  double m_arm_wrist_position (double p_position)
    {
        //
        // Ensure the specific value is legal.
        //
        l_arm_wrist_position = Range.clip
                ( p_position
                        , ArmWristServo_MinPosition
                        , ArmWristServo_MaxPosition
                );
        try {
            if (v_servo_arm_wrist != null) {
                v_servo_arm_wrist.setPosition(l_arm_wrist_position);
                return l_arm_wrist_position;
            } else {
                return ServoErrorResultPosition;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("arm_wrist", "missing", p_exeception);
            return ServoErrorResultPosition;
        }


    } // m_arm_elbow_position

*/

    /**
     * Mutate the flip right position.
     */
    /*double m_flip_right_position (double p_position)
    {
        //
        // Ensure the specific value is legal.
        //
        l_flip_right_position = Range.clip
                ( p_position
                        , FlipRightServo_MinPosition
                        , FlipRightServo_MaxPosition
                );
        try {
            if (v_servo_flip_right != null) {
                v_servo_flip_right.setPosition(l_flip_right_position);
                return l_flip_right_position;
            } else {
                return ServoErrorResultPosition;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("flip_right", "m_flip_right_position", p_exeception);
            return ServoErrorResultPosition;
        }
    }*/ // m_flip_right_position

    /**
     * Access the flip_right position.
     */
    /**
     * We use units of mm here because that's the recommended units of measurement for the
     * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
     *      <ImageTarget name="stones" size="247 173"/>
     * You don't *have to* use mm here, but the units here and the units used in the XML
     * target configuration files *must* correspond for the math to work out correctly.
     */
    static final float mmPerInch        = 25.4f;
    static final float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    static final float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
    private boolean v_vuforia_inited = false;
    private VuforiaLocalizer v_vuforia = null;
    private List<VuforiaTrackable> v_vuforia_allTrackables = null;
    private List<String> v_vuforia_messages = null;
    public enum vuforia_targets{
        Wheels (0,"Wheels"),
        Tools(1, "Tools"),
        Logos(2, "Logos"),
        Gears(3, "Gears"),
        BlueRedBeacon(4,"BlueRedBeacon"),
        RedBlueBeacon(5,"RedBlueBeacon");
        vuforia_targets(int targetIndex, String name) {
            this.targetIndex = targetIndex;
            this.name = name;
        }
        private final int targetIndex;   // in kilograms
        private final String name;

        public int targetIndex() { return targetIndex; }
        public static vuforia_targets getEnum(int targetIndex){
            switch (targetIndex){
                case 0:
                    return vuforia_targets.Wheels;
                case 1:
                    return vuforia_targets.Tools;
                case 2:
                    return vuforia_targets.Logos;
                case 3:
                    return vuforia_targets.Gears;
                case 4:
                    return vuforia_targets.BlueRedBeacon;
                case 5:
                    return vuforia_targets.RedBlueBeacon;

            }
            return null;
        }
    }
    public void vuforia_Init(){
            try{
                VuforiaLocalizer.Parameters parameters;
                if(v_debug) {
                     parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
                }else{
                    parameters = new VuforiaLocalizer.Parameters();
                }
                parameters.vuforiaLicenseKey = "ARr2v+H/////AAAAGbhUuRSeZUkynkK8ae61PIdjct7sVAoB5ItOs7Txvqsc9KlYRYHyftgUouhc+2Db+lSdUHCFdKp/CTYa3oWdQO3Bt3jkFplXQThhCFPnq0urXzwO0Mm5Jj1tYYuGZIU0anvdpA6DZVP95tL/FwRVO1BatviHrgurUy3L/TL7lPse5gI30PNKjgraalsKhmTxd13leA3dg+i/kqaTz3ot4iAmHEV6HBzsa3WUFSo1b6ig4Eo44j/O5J3CEQLWJYqRjlQwLUWB5QJi84YmhK2i+dSwdAXBc14Nb2QwsCjbbZA+XbSNxdDMKOTvCbVxHj+wL5Xare3nDZsPNTpEbKJ7ozaI7dcRJYCJK71X4Nv3fKn0";
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                v_vuforia = ClassFactory.createVuforiaLocalizer(parameters);

                // only care about 1 at the moment so commented this line
                // for performance Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
                /** For convenience, gather together all the trackable objects in one easily-iterable collection */
                v_vuforia_allTrackables = new ArrayList<VuforiaTrackable>();
                /**
                 * Load the data sets that for the trackable objects we wish to track. These particular data
                 * sets are stored in the 'assets' part of our application (you'll see them in the Android
                 * Studio 'Project' view over there on the left of the screen). You can make your own datasets
                 * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
                 * example "StonesAndChips", datasets can be found in in this project in the
                 * documentation directory.
                 */
                VuforiaTrackables ftc = v_vuforia.loadTrackablesFromAsset("FTC_2016-17");

                /**
                 * Create a transformation matrix describing where the phone is on the robot. Here, we
                 * put the phone on the right hand side of the robot with the screen facing in (see our
                 * choice of BACK camera above) and in landscape mode. Starting from alignment between the
                 * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
                 *
                 * When determining whether a rotation is positive or negative, consider yourself as looking
                 * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
                 * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
                 * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
                 * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
                 */
                OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                        .translation(0,0,0)
                        .multiplied(Orientation.getRotationMatrix(
                                AxesReference.EXTRINSIC, AxesOrder.YZY,
                                AngleUnit.DEGREES, 180, 0, 0));

                int trackableIndex = 0;
                for (VuforiaTrackable trackable : ftc) {
                    /**
                     * getUpdatedRobotLocation() will return null if no new information is available since
                     * the last time that call was made, or if the trackable is not currently visible.
                     * getRobotLocation() will return null if the trackable is not currently visible.
                     */
                    vuforia_targets myTarget = vuforia_targets.getEnum(trackableIndex);
                    trackable.setName(myTarget.name);
                    VuforiaTrackableDefaultListener myListener = (VuforiaTrackableDefaultListener)trackable.getListener();
                    /**
                     * Let the trackable listeners we care about know where the phone is. We know that each
                     * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
                     * we have not ourselves installed a listener of a different type.
                     */
                    myListener.setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
                    //trackable.setListener(myListener);
                    v_vuforia_allTrackables.add(trackable);
                    trackableIndex++;
                }
                /** Start tracking the FTC targets. */
                ftc.activate();

//                /*//Get our custom beacon trackables
//                ftc = v_vuforia.loadTrackablesFromAsset("FTC_Beacons");
//                for (VuforiaTrackable trackable : ftc) {
//                    *//**
//                     * getUpdatedRobotLocation() will return null if no new information is available since
//                     * the last time that call was made, or if the trackable is not currently visible.
//                     * getRobotLocation() will return null if the trackable is not currently visible.
//                     *//*
//                    vuforia_targets myTarget = vuforia_targets.getEnum(trackableIndex);
//                    trackable.setName(myTarget.name);
//                    VuforiaTrackableDefaultListener myListener = (VuforiaTrackableDefaultListener)trackable.getListener();
//                    *//**
//                     * Let the trackable listeners we care about know where the phone is. We know that each
//                     * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
//                     * we have not ourselves installed a listener of a different type.
//                     *//*
//                    myListener.setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//                    //trackable.setListener(myListener);
//                    v_vuforia_allTrackables.add(trackable);
//                    trackableIndex++;
//                }
//                *//** Start tracking our  FTC targets. *//*
//                ftc.activate();*/

                //Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image

//               v_vuforia..setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
//
///*To access the image: you need to iterate through the images of the frame object:*/
//
//                CloseableFrame frame = locale.getFrameQueue().take() //takes the frame at the head of the queue
//                Image rgb = null;
//
//                long numImages = frame.getNumImages();
//
//
//                for (int i = 0; i < numImages; i++) {
//                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                        rgb = frame.getImage(i);
//                        break;
//                    }//if
//                }//for


                v_vuforia_inited = true;
        }catch (Exception p_exeception)
        {
            debugLogException("vuforia_init", "Error", p_exeception);
            v_vuforia_inited = false;
        }
    }

    public boolean vuforia_targetVisible(int targetIndex){

        if(v_vuforia_inited == true){
            if(targetIndex < v_vuforia_allTrackables.size()){
                VuforiaTrackable trackable = v_vuforia_allTrackables.get(targetIndex);
                return ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible();
            }
        }
        return false;
    }
    //static final private float v_vuforia_driveToTarget_power_fast = .8f;
    //AndyMark 40 Worked in Coloma
    //static final private float v_vuforia_driveToTarget_power = .5f;   //.4f
    //static final private float v_vuforia_driveToTarget_power_slow = .2f; //.1

    private float v_vuforia_driveToTarget_power = .5f;   //.4f
    private float v_vuforia_driveToTarget_power_slow = .2f; //.1


    public void setup_am20(){
        //am20 run in reverse as am40 so swap reversed motor
        v_motor_right_drive.setDirection (DcMotor.Direction.FORWARD);
        v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
        v_drive_power = 0.8f;
        //we have to move slower backing up to prevent a wheely
        v_drive_power_reverse = 0.5f;
        v_drive_power_slowdown = .5f;
        v_turn_motorspeed = .3f;
        v_turn_motorspeed_slow = .25f;
        v_drive_inches_ticksPerInch = 45d;
        v_drive_turn_ticks_per_degree = 5.5f;
        v_vuforia_driveToTarget_power = .35f; //was .5f 12/16/2016
        v_vuforia_driveToTarget_power_slow = .15f;
        v_vuforia_driveToTarget_slowDownFactorClose = .009f;
         v_vuforia_driveToTarget_slowDownFactor = .020f;
        v_vuforia_findTargetSpeed = -.07f;
        v_drive_use_slowdown = false;
        v_drive_inches_slowdown = 24;
        v_drive_inches_slowdown_ticks = (int)Math.round(v_drive_inches_slowdown * v_drive_inches_ticksPerInch);
    }
    private int v_vuforia_driveToTarget_Index = -1;
    private ElapsedTime v_vuforia_driveToTarget_elapsedtime;
    public boolean vuforia_driveToTarget(int targetIndex){

        if(v_vuforia_inited == true){
            v_vuforia_driveToTarget_elapsedtime = new ElapsedTime();
            if(targetIndex < v_vuforia_allTrackables.size()){
                setupAutoDrive();
                v_vuforia_driveToTarget_Index = targetIndex;
                VuforiaTrackable trackable = v_vuforia_allTrackables.get(targetIndex);
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() == false){
                    return false;
                }else{

                    set_drive_power(v_vuforia_driveToTarget_power,v_vuforia_driveToTarget_power);
                    return true;
                }
            }
        }
        return false;
    }
    /*public boolean vuforia_driveToTargetFast(int targetIndex){

        if(v_vuforia_inited == true){
            if(targetIndex < v_vuforia_allTrackables.size()){
                setupAutoDrive();
                v_vuforia_driveToTarget_Index = targetIndex;
                VuforiaTrackable trackable = v_vuforia_allTrackables.get(targetIndex);
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() == false){
                    return false;
                }else{

                    set_drive_power(v_vuforia_driveToTarget_power_fast,v_vuforia_driveToTarget_power_fast);
                    return true;
                }
            }
        }
        return false;
    }*/

//    public void drive_setMaxSpeed(int tickPerSecond){
//        v_motor_left_drive.setMaxSpeed(tickPerSecond);
//        v_motor_right_drive.setMaxSpeed(tickPerSecond);
//    }
    static final private int v_vuforia_driveToTarget_xmin = -140; //130
    static final private int v_vuforia_driveToTarget_xmin_slow = -400;
    private float v_vuforia_driveToTarget_slowDownFactorClose = .007f; //130
    private float v_vuforia_driveToTarget_slowDownFactor = .020f;

    static float v_vuforia_findTargetSpeed = -.1f;

    public boolean vuforia_driveToTargetComplete(){
            if(v_vuforia_inited == true){

                VuforiaTrackable trackable = v_vuforia_allTrackables.get(v_vuforia_driveToTarget_Index);
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() == false){
                    if(v_vuforia_driveToTarget_elapsedtime.seconds() > 1 && v_vuforia_driveToTarget_elapsedtime.seconds() < 2) {
                        v_motor_right_drive.setPower(v_vuforia_findTargetSpeed);
                        v_motor_left_drive.setPower(0);
                    }else if(v_vuforia_driveToTarget_elapsedtime.seconds() > 2 && v_vuforia_driveToTarget_elapsedtime.seconds() < 4) {
                        v_motor_right_drive.setPower(0);
                        v_motor_left_drive.setPower(v_vuforia_findTargetSpeed);
                    }else {
                        set_drive_power(0f, 0f);
                    }
                    //set_first_message("Target Not Visable stoping");
                }else{
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();

                    if (pose != null) {
                        VectorF translation = pose.getTranslation();
                        double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                        float v_motorpower_left ;
                        float v_motorpower_right;
                        float v_motorpower_slowDownFactor;
                        float distanceX = translation.get(2);
                        if(distanceX > v_vuforia_driveToTarget_xmin){
                            set_drive_power(0.0f,0.0f);
                            //set_first_message("Target: " + trackable.getName() + ":" + distanceX  + ":" + degreesToTurn + ":" + translation.toString());
                            return true;
                        }
                        if(distanceX > v_vuforia_driveToTarget_xmin_slow){
                            v_motorpower_left = v_vuforia_driveToTarget_power_slow;
                            v_motorpower_right = v_vuforia_driveToTarget_power_slow;
                            v_motorpower_slowDownFactor = v_vuforia_driveToTarget_slowDownFactorClose;
                        }else{
                            v_motorpower_left = v_vuforia_driveToTarget_power;
                            v_motorpower_right = v_vuforia_driveToTarget_power;
                            v_motorpower_slowDownFactor = v_vuforia_driveToTarget_slowDownFactor;
                        }
                        if(degreesToTurn < 0 ){
                            float rightPowerAdjust = (180f - (float)(0-degreesToTurn)) * v_motorpower_slowDownFactor;
                            /*if(rightPowerAdjust > v_vuforia_maxslowDownFactor){
                                rightPowerAdjust = v_vuforia_maxslowDownFactor;
                            }*/
                            v_motorpower_right = (v_motorpower_right -rightPowerAdjust);
                        }else {
                            float leftPowerAdjust = (180f - (float)degreesToTurn) * v_motorpower_slowDownFactor;
                            /*if(leftPowerAdjust > v_vuforia_maxslowDownFactor){
                                leftPowerAdjust = v_vuforia_maxslowDownFactor;
                            }*/
                            v_motorpower_left = (v_motorpower_left - leftPowerAdjust);
                        }
                        set_drive_power ( v_motorpower_left,v_motorpower_right);
                        //set_first_message("searching: " + trackable.getName() + ":" + distanceX  + ":" + degreesToTurn + ":" + translation.toString() );
                    }
                }

            }
            return false;

    }


/*
    static final private int v_vuforia_driveToTargetFast_xmin = -2000;
    static final private int v_vuforia_driveToTargetFast_xmin_slow = -1350;
    public boolean vuforia_driveToTargetFastComplete(){
        if(v_vuforia_inited == true){

            VuforiaTrackable trackable = v_vuforia_allTrackables.get(v_vuforia_driveToTarget_Index);
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() == false){
                set_drive_power(0f,0f);
                //set_first_message("Target Not Visable stoping");
            }else{
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();

                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                    float v_motorpower_left ;
                    float v_motorpower_right;
                    float v_motorpower_slowDownFactor;
                    float distanceX = translation.get(2);
                    if(distanceX > v_vuforia_driveToTargetFast_xmin){
                        set_drive_power(0.0f,0.0f);
                        //set_first_message("Target: " + trackable.getName() + ":" + distanceX  + ":" + degreesToTurn + ":" + translation.toString());
                        return true;
                    }
                    if(distanceX > v_vuforia_driveToTargetFast_xmin_slow){
                        v_motorpower_left = v_vuforia_driveToTarget_power_slow;
                        v_motorpower_right = v_vuforia_driveToTarget_power_slow;
                        v_motorpower_slowDownFactor = .007f;
                    }else{
                        v_motorpower_left = v_vuforia_driveToTarget_power_fast;
                        v_motorpower_right = v_vuforia_driveToTarget_power_fast;
                        v_motorpower_slowDownFactor = .01f;
                    }
                    if(degreesToTurn < 0 ){
                        float rightPowerAdjust = (180f - (float)(0-degreesToTurn)) * v_motorpower_slowDownFactor;
                            */
/*if(rightPowerAdjust > v_vuforia_maxslowDownFactor){
                                rightPowerAdjust = v_vuforia_maxslowDownFactor;
                            }*//*

                        v_motorpower_right = (v_motorpower_right -rightPowerAdjust);
                    }else {
                        float leftPowerAdjust = (180f - (float)degreesToTurn) * v_motorpower_slowDownFactor;
                            */
/*if(leftPowerAdjust > v_vuforia_maxslowDownFactor){
                                leftPowerAdjust = v_vuforia_maxslowDownFactor;
                            }*//*

                        v_motorpower_left = (v_motorpower_left - leftPowerAdjust);
                    }
                    set_drive_power ( v_motorpower_left,v_motorpower_right);
                    //set_first_message("searching: " + trackable.getName() + ":" + distanceX  + ":" + degreesToTurn + ":" + translation.toString() );
                }
            }

        }
        return false;

    }
*/

    private void vuforia_hardwareLoop(){
        //doing nothing in here now but maybe update xyz later if we get that to work;
    }

    /**
     * Turn on the red led located in the Device Interface Module
     * @return returns true is successfull in turning on the led returns false on error
     */
    public boolean redled_on () {
        try {
            if (v_dim != null) {
                v_dim.setLED(1, true);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("dim redled", "redled_on", p_exeception);
            return false;
        }
    }

    /**
     * Turn off the red led located in the Device Interface Module
     * @return returns true is successfull in turning on the led returns false on error
     */
    public boolean redled_off () {
        try {
            if (v_dim != null) {
                v_dim.setLED(1, false);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("dim redled", "redled_off", p_exeception);
            return false;
        }
    }

    /**
     * Toggles the current state of the red led located in the Device Interface Module
     * <p>calling the function repeataly will give a blink effect.
     * @return returns true is successfull in turning on the led returns false on error
     */

    public boolean redled_toggle () {
        try {
            if (v_dim != null) {
                boolean isEnabled = v_dim.getLEDState(1);
                if (isEnabled) {
                    isEnabled = false;
                    set_second_message("Red Led set to Off");
                } else {
                    isEnabled = true;
                    set_second_message("Blue Led set to On");
                }
                v_dim.setLED(1, isEnabled);
                return isEnabled;
            }else {
                return false;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("dim redled", "redled_toggle", p_exeception);
            return false;
        }
    }

    /**
     * Turn on the blue led located in the Device Interface Module
     * @return returns true is successfull in turning on the led returns false on error
     */
    public boolean blueled_on () {
        try {
            if (v_dim != null) {
                v_dim.setLED(0, true);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("dim blueled", "blueled_on", p_exeception);
            return false;
        }
    }


    /**
     * Turn off the blue led located in the Device Interface Module
     * @return returns true is successfull in turning on the led returns false on error
     */
    public boolean blueled_off () {
        try {
            if (v_dim != null) {
                v_dim.setLED(0, false);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("dim blueled", "blueled_off", p_exeception);
            return false;
        }
    }



    /**
     * Toggle the blue led located in the Device Interface Module
     * @return returns true is successfull in turning on the led returns false on error
     */
    public boolean blueled_toggle () {
        try {
            if (v_dim != null) {
                boolean isEnabled = v_dim.getLEDState(0);
                if (isEnabled) {
                    isEnabled = false;
                    set_second_message("Blue Led set to Off");
                } else {
                    isEnabled = true;
                    set_second_message("Blue Led set to On");
                }
                v_dim.setLED(0, isEnabled);

                return isEnabled;
            } else {
                return false;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("dim blueled", "blueled_toggle", p_exeception);
            return false;
        }
    }



    /**
     * reset the gyro heading to zero
     */
    private boolean sensor_gyro_resetHeading(){
        try{
            if(v_sensor_gyro != null){
                // get the x, y, and z values (rate of change of angle).

                v_sensor_gyro.resetZAxisIntegrator();
                return true;
            }
            return false;
        }catch(Exception p_exeception)
        {
            debugLogException("sensor_gyro", "sensor_gyro_resetHeading", p_exeception);
            return false;
        }
    }

    public boolean beacon_make_red() throws InterruptedException{
        //the sensor is on the right side so the logic is  to read the
        //rgb sensor value if Red is higher then 1000 extend the right side pushing the button to make it
        //blue  else if Blue is over 1000 then extend the left button
        if(v_sensor_color_i2c_enabled){
            int v_detectedColor = sensor_color_GreatestColor();
            if(v_detectedColor == 0){
                set_second_message("Red Detected on Right Pushing Right Side");
                pushbutton_right_extend();
                ////////pushbutton_left_extend();
                timewait2Milliseconds(700);
                while (timewait2Milliseconds_Complete()==false){
                    hardware_loop();
                }
                pushbutton_right_retract();
                return true;
            }else if (v_detectedColor == 2){
                set_second_message("Blue Detected on Right Pushing Left Side");
                pushbutton_left_extend();
                /////////////pushbutton_right_extend();
                timewait2Milliseconds(700);
                while (timewait2Milliseconds_Complete()==false){
                    hardware_loop();
                }
                pushbutton_left_retract();
                return true;
            }else{
                //
                set_second_message("No Red or Blue Detected nothing to push");
            }
        }
        return false;
    }

    private int v_sensor_color_min_value = 500;

    //returns -1 if neither detected, 0 if red detected, 2 if blue detected higher
    public int sensor_color_GreatestColor(){
        if(v_sensor_color_i2c_enabled) {
            int[] myRGBA = sensor_color_get_rgba();
            //make sure at least one of them is over the min threshold else return -1
            int cRed = myRGBA[0];
            int cBlue = myRGBA[2];

            if (cRed < v_sensor_color_min_value && cBlue < v_sensor_color_min_value) {
                set_third_message("Both Red and Blue Under Min");
                return -1;
            }

            if (cRed > cBlue) {
                set_third_message("Red is Greater then Blue");
                return 0;
            } else {
                set_third_message("Blue is Greater then Red");
                return 2;
            }

        }
        return -1;
    }

    public int sensor_getGreatestColor(){
        if(v_sensor_color_i2c_enabled) {
            return sensor_color_GreatestColor();
        }else{
            return -1;
        }


    }

    public boolean beacon_make_blue() throws InterruptedException{
        //the sensor is on the right side so the logic is  to read the
        //rgb sensor value if Red is higher then 1000 extend the right side pushing the button to make it
        //blue  else if Blue is over 1000 then extend the left button
        if(v_sensor_color_i2c_enabled){
            int v_detectedColor = sensor_color_GreatestColor();
            if(v_detectedColor == 0){
                set_second_message("Red Detected on Right Pushing Left Side");
                pushbutton_left_extend();
                //////////pushbutton_right_extend();
                timewait2Milliseconds(700); //.7f
                while (timewait2Milliseconds_Complete()==false){
                    hardware_loop();
                }
                pushbutton_left_retract();
                return true;
            }else if (v_detectedColor == 2){
                set_second_message("Blue Detected on Right Pushing Right Side");
                pushbutton_right_extend();
                /////////////pushbutton_left_extend();
                timewait2Milliseconds(700); //.7
                while (timewait2Milliseconds_Complete()==false){
                    hardware_loop();
                }
                pushbutton_right_retract();
                return true;
            }else{
                //-1 returned so no color of threshold
                //
                set_second_message("No Red or Blue Detected nothing to push");
            }
        }
        return false;
    }

    /**
     * Enable the Color Sensor Led
     * @return returns true is successfull returns false on error
     */
    public boolean sensor_color_led(boolean enable){
        try{

            if(v_sensor_color_i2c !=null) {
                v_sensor_color_i2c_led_enabled = enable;
                v_dim.setDigitalChannelState(v_sensor_color_i2c_led_pin, v_sensor_color_i2c_led_enabled);
                //v_sensor_color_i2c.enableLed(enable);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_color", "sensor_color_led", p_exeception);
            return false;
        }
    }

    /**
     * Enable the Legecy Color Sensor
     * @return returns true is successfull returns false on error
     */
     public boolean sensor_color_enable(boolean enable){
        try{
            // convert the RGB values to HSV values.
            if(v_sensor_color_i2c !=null) {
                //turn on the led this is the only way legecy color will detect anything
                v_sensor_color_i2c_enabled = enable;
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_color", "sensor_color_enable", p_exeception);
            return false;
        }
    }


    public int[] sensor_color_get_rgba(){
        try{
            return v_sensor_color_i2c_rgbaValues;

        }catch (Exception p_exeception)
        {
            debugLogException("sensor_color", "sensor_color_read_rgb", p_exeception);
            return v_sensor_color_i2c_rgbaValues;
        }
    }



    /**
     *
     * @return gyro heading in degrees since reset
     */
    public int sensor_gyro_get_heading(){
        try{
            if(v_sensor_gyro != null) {
                return v_sensor_gyro.getHeading();
            }else{
                return 0;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_gyro", "sensor_gyro_get_heading", p_exeception);
            return 0;
        }


    }

    /**
     * return the rawX rate
     * @return gyro heading in degrees since reset
     */
    public int sensor_gyro_get_rawX(){
        try{
            // get the x info.
            if(v_sensor_gyro != null) {
                return v_sensor_gyro.rawX();
            }else{
                return 0;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_gyro", "sensor_gyro_get_rawX", p_exeception);
            return 0;
        }

    }

    /**
     * return the rawX rate
     * @return gyro heading in degrees since reset
     */
    public int sensor_gyro_get_rawY(){
        try{
            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            if(v_sensor_gyro != null) {
                return v_sensor_gyro.rawY();
            }
            else{
                return 0;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_gyro", "sensor_gyro_get_rawY", p_exeception);
            return 0;
        }
    }

    /**
     * return the rawX rate
     * @return gyro heading in degrees since reset
     */
    public int sensor_gyro_get_rawZ(){
        try{
            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            if(v_sensor_gyro != null) {
                return v_sensor_gyro.rawZ();
            }
            else{
                return 0;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_gyro", "sensor_gyro_get_rawZ", p_exeception);
            return 0;
        }
    }




    /**
     * Enable the Legecy Color Sensor
     * @return returns true is successfull returns false on error
     */
/*
    public boolean sensor_colorLegecy_start(){
        try{
            // convert the RGB values to HSV values.
            if(v_sensor_colorLegecy_rgbValues !=null) {
                //turn on the led this is the only way legecy color will detect anything
                sensor_colorLegecy_led(true);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_start", p_exeception);
            return false;
        }
    }
*/
    /*public int[] sensor_colorLegecy_getLast_rgba(){
        try{
            return v_sensor_colorLegecy_rgbValues;

        }catch (Exception p_exeception)
        {
            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_getLast_rgb", p_exeception);
            return v_sensor_colorLegecy_rgbValues;
        }
    }


    public double sensor_ultraLegecy_distance(){
        try{
            if(v_sensor_ultraLegecy != null){
                if ((v_loop_ticks % v_sensor_ultraLegecy_ticksPerRead) == 0) {
                    v_sensor_ultraLegecy_distance = v_sensor_ultraLegecy.getUltrasonicLevel();
                }
                return v_sensor_ultraLegecy_distance;
            }else{
                return 9999.9999;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_ultraLegecy", "sensor_ultraLegecy_distance", p_exeception);
            return 9999.9999;
        }
    }
*/
    //Lego Light Legecy Sensor Methods

    //--------------------------------------------------------------------------

    //
    // a_ods_light_detected
    /**
     * Disables the Legecy Color Sensor
     * @return returns true is successfull returns false on error
     */
/*
    public boolean sensor_colorLegecy_stop(){
        try{
            // convert the RGB values to HSV values.
            if(v_sensor_colorLegecy_rgbValues !=null) {
                //turn on the led this is the only way legecy color will detect anything
                sensor_colorLegecy_led(false);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_stop", p_exeception);

            return false;
        }
    }
*/

/*
    private int[] sensor_colorLegecy_read_rgba(){
        try{
            // convert the RGB values to HSV values.
            if(v_sensor_colorLegecy_rgbValues !=null) {
                //v_sensor_color.enableLed(true);
                // wait one cycle.
                //waitOneFullHardwareCycle();
                v_sensor_colorLegecy_rgbValues[0] = v_sensor_colorLegecy.red();
                v_sensor_colorLegecy_rgbValues[1] = v_sensor_colorLegecy.green();
                v_sensor_colorLegecy_rgbValues[2] = v_sensor_colorLegecy.blue();
                v_sensor_colorLegecy_rgbValues[3] = v_sensor_colorLegecy.alpha();
                // wait one cycle.
                //waitOneFullHardwareCycle();
                // v_sensor_color.enableLed(false);
            }
            //Color.RGBToHSV(v_sensor_color.red(), v_sensor_color.green(), v_sensor_color.blue(), v_sensor_color_hsvValues);
            return v_sensor_colorLegecy_rgbValues;

        }catch (Exception p_exeception)
        {
            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_read_rgb", p_exeception);
            return v_sensor_colorLegecy_rgbValues;
        }
    }
*/
    //
    /**
     * Access the amount of light detected by the Optical Distance Sensor.
     */
/*
    public double sensor_lightLegecy_amountDetected ()

    {
        double l_return = 0.0;

        if (v_sensor_lightLegecy != null)
        {
            v_sensor_lightLegecy.getLightDetected ();
        }

        return l_return;

    }
    public boolean sensor_lightLegecy_led(boolean enable){
        if(v_sensor_lightLegecy != null) {
            v_sensor_lightLegecy_enabled = enable;
            v_sensor_lightLegecy.enableLed(enable);
            return true;
        }else{
            return false;
        }
    }

    public boolean sensor_lightLegecy_led_status(){
        return v_sensor_lightLegecy_enabled;
    }
    public boolean sensor_lightLegecy_white_tape_detected(){
        return a_light_white_tape_detected();
    }

*/
    //--------------------------------------------------------------------------
    //
    // a_light_white_tape_detected
    //
    /**
     * Access whether the Light Sensor is detecting white tape.
     */
/*
    private boolean a_light_white_tape_detected ()
    {

        //
        // Assume not.
        //
        boolean l_return = false;

        if (v_sensor_lightLegecy != null)
        {
            //
            // Is the amount of light detected above the threshold for white
            // tape?
            //
            if (v_sensor_lightLegecy.getLightDetected () > 0.8)
            {
                l_return = true;
            }
        }

        //
        // Return
        //
        return l_return;

    } // a_ods_white_tape_detected
*/

    //Don't use these inless we are in linerOpMode
//    public void waitOneFullHardwareCycle() throws InterruptedException {
//        this.waitForNextHardwareCycle();
//        Thread.sleep(1L);
//        this.waitForNextHardwareCycle();
//    }
//
//    public void waitForNextHardwareCycle() throws InterruptedException {
//        synchronized(this) {
//            this.wait();
//        }
//    }
//
    public void sleep(long milliseconds) {
        try{

            Thread.sleep(milliseconds);
        }catch(InterruptedException ex){
            //do stuff
        }
    }
    
    //Below are The Telmetry Code to Write Debug to the Phones

    String secondMessage = "N/A";


    //--------------------------------------------------------------------------
    //
    // update_telemetry
    //
    /**
     * Update the telemetry with current values from the base class.
     */
    private void update_telemetry ()

    {
        try {
            opMode.telemetry.addData("00", loopCounter() + ":" + hardware_loop_slowtime_milliseconds() + ":"+ a_warning_message());
            opMode.telemetry.addData("01", zeroMessage);
            opMode.telemetry.addData("02", firstMessage);
            opMode.telemetry.addData("03", secondMessage);
            opMode.telemetry.addData("04",  thirdMessage);
            opMode.telemetry.addData("05", "Gyro: H:" + sensor_gyro_get_heading() + ",X:" + sensor_gyro_get_rawX() + ",Y:" + sensor_gyro_get_rawY() + ",Z:" + sensor_gyro_get_rawZ());
            if (v_debug) {
                //
                // Send telemetry data to the driver station.
                //
                opMode.telemetry.addData
                        ("06"
                                , "Left Drive: "
                                        + a_left_drive_power()
                                        + ", "
                                        + a_left_encoder_count()
                                        + ", "
                                        + a_left_drive_mode()
                        );
                opMode.telemetry.addData
                        ("07"
                                , "Right Drive: "
                                        + a_right_drive_power()
                                        + ", "
                                        + a_right_encoder_count()
                                        + ", "
                                        + a_right_drive_mode()
                        );
                if(v_motor_rackpinion != null) {
                    opMode.telemetry.addData("rackpinion", " to %7d at %7d",
                            v_motor_rackpinion_Position,
                            v_motor_rackpinion.getCurrentPosition());
                }
                if(v_sensor_color_i2c_enabled) {
                    int[] v_color_rgba = sensor_color_get_rgba();
                    opMode.telemetry.addData(
                            "color", "Color RGBA: " + v_color_rgba[0]
                                    + "," + v_color_rgba[1]
                                    + "," + v_color_rgba[2]
                                    + "," + v_color_rgba[3]
                    );
                }
//                opMode.telemetry.addData
//                        ("06"
//                                , "RPA Base Position: " //+ a_rpabase_position()
//                        );
//                opMode.telemetry.addData
//                        ("07"
//                                , "RPA Arm Position: " //+ a_rpa_arm_power() + ":" + rpa_arm_extended() + ":" + rpa_arm_retracted()
//                        );
//                opMode.telemetry.addData(
//                        "08", "Flip: Right:" //+ a_flip_right_position()
//                );
                /*opMode.telemetry.addData
                        ("05"
                                , "Arm Shoulder: " + a_arm_shoulder_position()
                        );
                opMode.telemetry.addData
                        ("06"
                                , "Arm Elbow: " + a_arm_elbow_position()
                        );
                opMode.telemetry.addData
                        ("07"
                                , "Arm Wrist: " + a_arm_wrist_position()
                        );



                opMode.telemetry.addData(
                        "1l", "Flip: Right:" + a_flip_right_position() + ", Left:" + a_flip_left_position()
                );
                opMode.telemetry.addData(
                        "12", "Ultra: " + sensor_ultraLegecy_distance()
                );
                opMode.telemetry.addData(
                        "13", "Light: tape:" + sensor_lightLegecy_white_tape_detected() + "," + sensor_lightLegecy_amountDetected()
                );*/
                if(v_vuforia_inited == true){
                    int position = 0;
                    for (VuforiaTrackable trackable : v_vuforia_allTrackables) {
                        /**
                         * getUpdatedRobotLocation() will return null if no new information is available since
                         * the last time that call was made, or if the trackable is not currently visible.
                         * getRobotLocation() will return null if the trackable is not currently visible.
                         */

                        opMode.telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");

                        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();

                        if (pose != null) {
                            VectorF translation = pose.getTranslation();
                            double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                            opMode.telemetry.addData(trackable.getName() + "-Degrees", degreesToTurn);
                            /*double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(1)));
                            double degreesToTurn2 = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                            double degreesToTurn3 = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                            opMode.telemetry.addData(trackable.getName() + "-Degrees", degreesToTurn);
                            opMode.telemetry.addData(trackable.getName() + "-Degrees2", degreesToTurn2);
                            opMode.telemetry.addData(trackable.getName() + "-Degrees3", degreesToTurn3);*/
                        }
                    }
                }
            }
        }catch (Exception p_exeception)
        {
            set_first_message("updateTelmetry: " + p_exeception.getLocalizedMessage());
        }
    } // update_telemetry

    //--------------------------------------------------------------------------
    //
    // update_gamepad_telemetry
    //
    /**
     * Update the telemetry with current gamepad readings.
     */
    public void update_gamepad_telemetry ()

    {
        //
        // Send telemetry data concerning gamepads to the driver station.
        //
        if (v_debug) {
            opMode.telemetry.addData("14", "GP1 Left: " + -opMode.gamepad1.left_stick_y);
            opMode.telemetry.addData("15", "GP1 Right: " + -opMode.gamepad1.right_stick_y);
            opMode.telemetry.addData("16", "GP2 Left: " + -opMode.gamepad2.left_stick_y);
            opMode.telemetry.addData("17", "GP2 X: " + opMode.gamepad2.x);
            opMode.telemetry.addData("18", "GP2 Y: " + opMode.gamepad2.y);
            opMode.telemetry.addData("19", "GP2 A: " + opMode.gamepad2.a);
            opMode.telemetry.addData("20", "GP1 LT: " + opMode.gamepad1.left_trigger);
            opMode.telemetry.addData("21", "GP1 RT: " + opMode.gamepad1.right_trigger);
        }
    } // update_gamepad_telemetry

    String zeroMessage = "";
    public void set_message (String p_message)

    {
        zeroMessage = p_message;
        if (v_debug) {
            //DbgLog.msg(loopCounter() + ":0:" + p_message);
            android.util.Log.d("CFPushBotHardware",loopCounter() + ":0:" + p_message);
        }
    } // set_first_message
    //--------------------------------------------------------------------------
    //
    // set_first_message
    //
    /**
     * Update the telemetry's first message with the specified message.
     */
    String firstMessage = "";
    private void set_first_message (String p_message)
    {

        firstMessage = p_message;
        if (v_debug) {
            //DbgLog.msg(loopCounter() + ":1:" + p_message);
            android.util.Log.d("CFPushBotHardware",loopCounter() + ":1:" + p_message);
        }
    } // set_first_message

    //--------------------------------------------------------------------------
    //
    // set_second_message
    //
    /**
     * Update the telemetry's first message with the specified message.
     */
    private void set_second_message (String p_message)

    {
        secondMessage = p_message;
        if (v_debug) {
            //DbgLog.msg(loopCounter() + ":2:" + p_message);
            android.util.Log.d("CFPushBotHardware",loopCounter() + ":2:" + p_message);
        }

    }
    //--------------------------------------------------------------------------
    //
    // set_second_message
    //
    /**
     * Update the telemetry's first message with the specified message.
     */
    String thirdMessage = "";
    private void set_third_message (String p_message)

    {
        thirdMessage = p_message;
        if (v_debug) {
            android.util.Log.d("CFPushBotHardware",loopCounter() + ":3:" + p_message);
            //DbgLog.msg(loopCounter() + ":3:" + p_message);
        }

    }

    // set_first_message
    //--------------------------------------------------------------------------
    //
    // set_error_message
    //
    /**
     * Update the telemetry's first message to indicate an error.
     */
    public void set_error_message (String p_message)

    {
        set_first_message ("ERROR: " + p_message);

    } // set_error_message


    private ElapsedTime period  = new ElapsedTime();
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    private void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
