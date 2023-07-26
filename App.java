package com.nelli;

import ev3dev.sensors.slamtec.RPLidarA1;
import ev3dev.sensors.slamtec.RPLidarA1ServiceException;
import ev3dev.sensors.slamtec.RPLidarProviderListener;
import ev3dev.sensors.slamtec.model.Scan;
import ev3dev.sensors.slamtec.model.ScanDistance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class App {
    static final int forwardDirection = 90;
    static final int scanningRangeWidth = 30;
    static final int criticalDistance = 10;

    static NetworkTableEntry tooClose;
    public static void main(String[] args) throws RPLidarA1ServiceException, InterruptedException
    {      
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("lidar");
        tooClose = table.getEntry("tooClose");
        tooClose.setBoolean(false);
        inst.startClientTeam(1477);

        float[] distances = new float[360];   

        System.out.println("Connecting to LIDAR Device...");

        final String USBPort = "/dev/ttyUSB0";
        final RPLidarA1 lidar = new RPLidarA1(USBPort);     

        lidar.addListener(new RPLidarProviderListener(){
            @Override
            public void scanFinished(Scan scan) {
                System.out.println("Measures: "+ scan.getDistances().size());
                for(int i=0;i<360;i++)
                {
                    distances[i] = 0;
                }
                for (ScanDistance dis : scan.getDistances()) {
                    if(dis.getQuality()>0 && dis.getAngle()<360)
                    {
                        distances[dis.getAngle()] = dis.getDistance();
                    }
                }  
                updateTables(distances);              
            }           
        });

        lidar.init();
        System.out.println("Beginning scans...");

        while(true)
        {
            lidar.scan();
        }
    }

    private static void updateTables(float[] distances)
    {
        for(int i = forwardDirection-scanningRangeWidth; i < forwardDirection+scanningRangeWidth; i++)
        {
            if(distances[i]>0 && distances[i]<criticalDistance)
            {
                tooClose.setBoolean(true);
                return;
            }
        }
        tooClose.setBoolean(false);
    } 
}
