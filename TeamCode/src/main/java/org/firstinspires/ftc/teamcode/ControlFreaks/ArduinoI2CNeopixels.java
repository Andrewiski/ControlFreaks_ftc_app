package org.firstinspires.ftc.teamcode.ControlFreaks;

//import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by adevries on 11/16/2015.
 */
public class ArduinoI2CNeopixels {
    static final String logId   = "ArduinoI2CNeoPixels:";       // Tag identifier in FtcRobotController.LogCat
    private String v_deviceName = "Not Set";
    private byte I2CAddress = 0x26;
    private Wire v_neopixels;

    private byte v_mode = 0;
    private byte v_red = 0x00;
    private byte v_green = 0x00;
    private byte v_blue = (byte)0xFF;
    private byte v_brightness = (byte)0xFF;
    private boolean v_neopixels_enabled = false;
    public ArduinoI2CNeopixels(HardwareMap hardwareMap, String deviceName, byte RealI2CAddress) throws Exception
    {
        try{
            I2CAddress = RealI2CAddress;
            v_deviceName = deviceName;
            v_neopixels = new Wire(hardwareMap,deviceName,I2CAddress * 2);
            init();
        }catch (Exception p_exeception)
        {
            debugLogException("Error Creating Wire NeoPixels", p_exeception);
            v_neopixels = null;
            throw p_exeception;
        }
    }
    public ArduinoI2CNeopixels(HardwareMap hardwareMap, String deviceName) throws Exception

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.
        try{
            v_deviceName = deviceName;
            v_neopixels = new Wire(hardwareMap,deviceName,I2CAddress * 2);
            init();

        }catch (Exception p_exeception)
        {
            debugLogException("Error Creating Wire", p_exeception);
            v_neopixels = null;
        }

    }

    private void init() throws Exception{
        try{
            if (v_neopixels != null) {
                v_neopixels.beginWrite(0x00);
                //v_neopixels.write(0x00);
                v_neopixels.write(v_mode); //mode All lights off
                v_neopixels.write(v_red); // red
                v_neopixels.write(v_green); // green
                v_neopixels.write(v_blue); // blue
                v_neopixels.write(v_brightness); // brightness
                v_neopixels.endWrite();
                v_neopixels_enabled = true;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("Error Init", p_exeception);
            throw p_exeception;
        }
    }


    public boolean set_brightness(byte level) {
        v_brightness = level;
        if (v_neopixels != null) {

            v_neopixels.beginWrite(0x04); // brightness reg is at 4
            v_neopixels.write(v_brightness); // brightness
            v_neopixels.endWrite();
            return true;
        }else{
            return false;
        }
    }

    public boolean set_rgb(byte red, byte green, byte blue) {
        if (v_red == red && v_green == green && v_blue == blue){
            return false;
        }else{
            v_red = red;
            v_green = green;
            v_blue = blue;
            if (v_neopixels != null) {

                v_neopixels.beginWrite(0x01); // red reg is at 1
                v_neopixels.write(v_red); // red
                v_neopixels.write(v_green); // green
                v_neopixels.write(v_blue); // blue
                v_neopixels.endWrite();
                return true;
            }else{
                return false;
            }
        }
    }

    public boolean set_mode(byte mode) {
        if (v_mode == mode){
            return false;
        }else {
            if (v_neopixels != null) {
                v_neopixels.beginWrite(0x00); // mode reg is at 0
                v_neopixels.write(mode); // red
                v_neopixels.endWrite();
                return true;
            } else {
                return false;
            }
        }
    }


    public boolean isEnabled(){
        return v_neopixels_enabled;
    }



    void debugLogException( String msg, Exception ex){

        String debugMessage = logId + msg;
        if (ex != null) {
            String errMsg = ex.getLocalizedMessage();
            if (errMsg != null) {
                debugMessage = debugMessage + errMsg;
            }else{
                debugMessage = debugMessage + " error. is null";
            }
        }else{
            debugMessage = debugMessage + " error is null";
        }
        android.util.Log.d("ArduinoI2CNeopixels",debugMessage);
        //DbgLog.msg(debugMessage);
        //telemetry.addData(line, debugMessage);
    }
}
