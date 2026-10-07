package org.firstinspires.ftc.teamcode._2627.config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class PinpointConfig {
    /**
     * strafe offset
     */
    public double offsetX;
    /**
     * forward offset
     */
    public double offsetY;
    public String id;
    public GoBildaPinpointDriver.EncoderDirection Xdirection;
    public GoBildaPinpointDriver.EncoderDirection Ydirection;
    public PinpointConfig(String id, GoBildaPinpointDriver.EncoderDirection x_direction, GoBildaPinpointDriver.EncoderDirection y_direction, double offsetX, double offsetY){
        this.id = id;
        this.Xdirection = x_direction;
        this.Ydirection = y_direction;
        this.offsetX = offsetX;
        this.offsetY = offsetY;
    }
}
