package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pixy extends Module {
	// TODO: USE THIS CODE INSTEAD: https://github.com/BHSRobotix/Steamworks2017/blob/master/src/org/usfirst/frc2876/Steamworks2017/subsystems/PixyI2C.java
	private static final Port port = Port.kOnboard;

	PixyPacket values;
	I2C pixy;
	PixyPacket[] packets;

	public Output<PixyPacket> detections;

	public Pixy() {
		this(0x54);
	}

	public Pixy(int address) {
		this( new I2C(port, address), new PixyPacket[7], new PixyPacket());
	}

	public Pixy( I2C pixy, PixyPacket[] pixyPacket, PixyPacket values ) {
		super(Pixy.class);

		this.pixy = pixy;
		this.packets = pixyPacket;
		this.values = values;

		detections = addOutput("Pixy Scans", this::getResults);

		setDefaultAction(scan);
	}

	// This method parses raw data from the pixy into readable integers
	private int cvt(byte upper, byte lower) {
		return (((int)upper & 0xff) << 8) | ((int)lower & 0xff);
	}

	// This method gathers data, then parses that data, and assigns the ints to global variables
	public PixyPacket readPacket(int Signature) throws PixyException { //The signature should be which number object in 
		int Checksum;												   //pixymon you are trying to get data for
		int Sig;
		byte[] rawData = new byte[32];

		SmartDashboard.putString("rawData", rawData[0] + " " + rawData[1] + " " + rawData[15] + " " + rawData[31]);

		try{
			pixy.readOnly(rawData, 32);	
		} catch (RuntimeException e) {
			SmartDashboard.putString("Pixy RuntimeException", "Error");
		}

		if(rawData.length < 32){
			System.out.println("byte array length is broken");
			return null;
		}

		for (int i = 0; i <= 16; i++) {
			int syncWord = cvt(rawData[i+1], rawData[i+0]); //Parse first 2 bytes

			if (syncWord == 0xaa55) { //Check is first 2 bytes equal a "sync word", which indicates the start of a packet of valid data
				syncWord = cvt(rawData[i+3], rawData[i+2]); //Parse the next 2 bytes

				if (syncWord != 0xaa55){ //Shifts everything in the case that one syncword is sent
					i -= 2;
				}

				//This next block parses the rest of the data
				Checksum = cvt(rawData[i+5], rawData[i+4]);
				Sig = cvt(rawData[i+7], rawData[i+6]);
				if(Sig <= 0 || Sig > packets.length){
					break;
				}
				
				packets[Sig - 1] = new PixyPacket();
				packets[Sig - 1].X = cvt(rawData[i+9], rawData[i+8]);
				packets[Sig - 1].Y = cvt(rawData[i+11], rawData[i+10]);
				packets[Sig - 1].Width = cvt(rawData[i+13], rawData[i+12]);
				packets[Sig - 1].Height = cvt(rawData[i+15], rawData[i+14]);

				//Checks whether the data is valid using the checksum *This if block should never be entered*
				if (Checksum != Sig + packets[Sig - 1].X + packets[Sig - 1].Y + packets[Sig - 1].Width + packets[Sig - 1].Height) {
					packets[Sig - 1] = null;
					throw new PixyException("Pixy camera checksum failed! This *should not* happen.");
				}

				break;
			}
			else {
				SmartDashboard.putNumber("syncword: ", syncWord);
			}
		}

		//Assigns our packet to a temp packet, then deletes data so that we dont return old data
		PixyPacket pkt = packets[Signature - 1];
		packets[Signature - 1] = null;
		return pkt;
	}

	private class Idle extends Action {
		public Idle() {
			super(Pixy.this, Idle.class);
		}
	}

	private class Scan extends Action {
		public Scan() {
			super(Pixy.this, Scan.class);
		}

		@Override
		public void run() {
			for( int i = 0; i < packets.length; ++i ) {
				// Null any packets so that missed packets are not filled with old data
				packets[i] = null;
			}

			for( int i = 0; i < packets.length; ++i ) {
				packets[i] = readPacket(i);

				if( packets[i] == null ) {
					//throw new PixyException("Read a null packet. That shouldn't happen...");
				}



			}
		}
	}

	public Action idle = new Idle();
	public Action scan = new Scan();

	public int getX(){
		return values.X;
	}
	
	public int getY(){
		return values.Y;
	}
	
	public int getWidth(){
		return values.Width;
	}
	
	public int getHeight(){
		return values.Height;
	}

	public PixyPacket getResults() {
		return packets[0];
	}

	public static class PixyPacket {
		public int X;
		public int Y;
		public int Width;
		public int Height;
		public int checksumError;
	}

	public static class PixyException extends Exception {

		public PixyException(String message){
				super(message);
			}

	}
}

