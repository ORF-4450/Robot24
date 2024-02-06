package Team4450.Robot24.subsystems;

import static Team4450.Robot24.Constants.*;

import java.util.ArrayList;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;

import Team4450.Lib.Util;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase 
{
	private int				targetX = 0, targetY = 0, targetWidth = 0, targetHeight = 0;
	private int				xResolution = 320, yResolution = 240;
	private boolean			targetFound;
	
	// LimeLight variables read from network table.

	private NetworkTable	networkTable = NetworkTableInstance.getDefault().getTable("limelight");
	
	private double			xOffset, yOffset, area, skew, pipeLine, latency;

	NetworkTableEntry		tv = networkTable.getEntry("tv");
	NetworkTableEntry		tx = networkTable.getEntry("tx");
	NetworkTableEntry		ty = networkTable.getEntry("ty");
	NetworkTableEntry		ta = networkTable.getEntry("ta");
	NetworkTableEntry		pl = networkTable.getEntry("getpipe");
	NetworkTableEntry		ts = networkTable.getEntry("ts");
	NetworkTableEntry		tl = networkTable.getEntry("tl");
	NetworkTableEntry		thor = networkTable.getEntry("thor");
	NetworkTableEntry		tvert = networkTable.getEntry("tvert");
	NetworkTableEntry		snapshot = networkTable.getEntry("snapshot");
	NetworkTableEntry		tx0 = networkTable.getEntry("tx0");
	NetworkTableEntry		ty0 = networkTable.getEntry("ty0");
	NetworkTableEntry		ledMode = networkTable.getEntry("ledMode");
	NetworkTableEntry		camMode = networkTable.getEntry("camMode");
	NetworkTableEntry		pipeline = networkTable.getEntry("pipeline");
	NetworkTableEntry		streamMode = networkTable.getEntry("stream");

	public enum LedMode
	{
		pipeline,
		off,
		blink,
		on;
	}
	
	public enum CameraMode
	{
		vision,
		driver;
	}
	
	public enum StreamMode
	{
		standard,
		pipMain,
		pipSecondary;
	}
	
	public LimeLight() 
	{
		Util.consoleLog("LimeLight created!");

		setLedMode(LedMode.off);
	}
	
	/**
	 * Save snapshot to LimeLight storage.
	 * @param save True to save one snapshot.
	 */
	public void saveSnapshots(boolean save)
	{
		if (save)
			snapshot.setDouble(1);
		else
			snapshot.setDouble(0);
	}
	
	// This method will be called once per scheduler run by the scheduler.
	@Override
	public void periodic() 
	{
		// Commands that use this class will control the LED.
        // if (robot.isEnabled())
        //     setLedMode(LedMode.on);
        // else
        //     setLedMode(LedMode.off);
    }

	/**
	 * Update the targeting information held in this class from the LimeLight via the
	 * network tables.
	 * @return True if target detected, false is not.
	 */
	public boolean processImage()
	{
		double targetVisible = tv.getDouble(0);
    
        Util.consoleLog("tv=%.0f", targetVisible);

		if (targetVisible != 0)
			targetFound = true;
		else
			targetFound = false;
		
		xOffset = tx.getDouble(0);
		yOffset = ty.getDouble(0);
		area = ta.getDouble(0);
		pipeLine = pl.getDouble(0);
		skew = ts.getDouble(0);
		latency = tl.getDouble(0);
		targetWidth = (int) thor.getDouble(0);
		targetHeight = (int) tvert.getDouble(0);
		
		targetX = deNormalizeScreenSpaceX(tx0.getDouble(0));
		targetY = deNormalizeScreenSpaceY(ty0.getDouble(0));
		
		return targetFound;
	}

	/**
	 * Convert X axis normalized screen space (-1 to +1 mapping to 0 to 320) to pixels.
	 * @param value Normalized screen space value.
	 * @return Pixels from left edge.
	 */
	private int deNormalizeScreenSpaceX(double value)
	{
		int pixels = (int) (Math.abs(value) * (xResolution / 2));
		
		if (value >= 0)
			pixels += xResolution / 2;
		else
			pixels = (xResolution / 2) - pixels;
		
		return pixels;
	}

	/**
	 * Convert Y axis normalized screen space (-1 to +1 mapping to 0 to 320) to pixels.
	 * @param value Normalized screen space value.
	 * @return Pixels from top edge.
	 */
	private int deNormalizeScreenSpaceY(double value)
	{
		int pixels = (int) (Math.abs(value) * (yResolution / 2));
		
		if (value >= 0)
			pixels += yResolution / 2;
		else
			pixels = (yResolution / 2) - pixels;
		
		return pixels;
	}
	
	/**
	 * Indicates if a target was detected at the last call to processImage().
	 * @return True if a target was detected, false if not.
	 */
	public boolean targetVisible()
	{
		return targetFound; 
	}

	/**
	 * Return a rectangle bounding the target in the field of view.
	 * @return Rectangle with values in pixels.
	 */
	public Rect getTargetRectangle()
	{
		return new Rect(targetX, targetY, targetWidth, targetHeight);
	}

	public ArrayList<MatOfPoint> getContours()
	{
		return null;
	}

	/**
	 * Return a measure of the target area (size). 
	 * @return The distance as a % of the total field view area that
	 * is occupied by the target area.
	 */
	public double getArea()
	{
		return area;
	}
	/**
	 * Return a measure of the target distance. 
	 * @return The distance as a % of the total field view area that
	 * is occupied by the target area.
	 */
	public double getDistance()
	{
		return area;
	}

	/**
	 * Return a measure of the target distance. 
	 * @return The distance as an offset from the Y cursor setpoint.
	 * Assumes the Y cursor has been calibrated for the correct target
	 * distance. Will return zero when at correct distance to target.
	 */
	public double getDistanceY()
	{
		return yOffset;
	}

	/**
	 * Return X axis center of target in pixels from left edge.
	 * @return X center of target in pixels.
	 */
	public int centerX()
	{
		return targetX + (targetWidth / 2);
	}

	/**
	 * Return Y axis center of target in pixels from top edge.
	 * @return Y center of target in pixels.
	 */
	public int centerY()
	{
		return targetY + (targetHeight / 2);
	}

	/**
	 * Return the target center X axis offset from the center of the field of view.
	 * Note, center of field of view is the location of the cursor.
	 * @return The offset in degrees. - target is left of center, + target is
	 * right of center.
	 */
	public double offsetX()
	{
		return xOffset;
	}

	/**
	 * Return the target center Y axis offset from the center of the field of view.
	 * Note, the center of field of view is the location of the cursor.
	 * @return The offset in degrees. - target is below center, + target is above center.
	 */
	public double offsetY()
	{
		return  yOffset;
	}

	/**
	 * Set the resolution to be used in calculations.
	 * @param x X axis resolution.
	 * @param y Y axis resolution.
	 */
	public void setResolution(int x, int y)
	{
		xResolution = x;
		yResolution = y;	
	}
	
	/**
	 * Set the camera led mode of operation.
	 * @param mode Led mode.
	 */
	public void setLedMode(LedMode mode)
	{
		ledMode.setDouble((double) mode.ordinal());
	}
	
	/**
	 * Set the camera mode of operation.
	 * @param mode Camera mode. 
	 */
	public void setCameraMode(CameraMode mode)
	{
		camMode.setDouble((double) mode.ordinal());
	}
	
	/**
	 * Sets the streaming mode of the camera feed
	 * @param mode Stream mode.
	 */
	public void setStreamMode(StreamMode mode)
	{
		streamMode.setDouble((double) mode.ordinal());
	}
	
	/**
	 * Selects the pipeline in the LL memory as active.
	 * @param pipeline Pipeline number.
	 */
	public void selectPipeline(int pipeline)
	{
		this.pipeline.setDouble((double) pipeline);
	}
	
	/**
	 * Returns the active pipeline number.
	 * @return The active pipeline.
	 */
	public int getPipeline()
	{
		return (int) pipeLine;
	}
	
	/**
	 * Returns the skew angle.
	 * @return Skew angle -90 to 0.
	 */
	public int getSkew()
	{
		return (int) skew;
	}
	
	/**
	 * Pipeline processing time.
	 * @return Processing time (latency) in ms.
	 */
	public int getLatency()
	{
		return (int) latency;
	}
}
