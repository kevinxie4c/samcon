package odemocap;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Scanner;

/**
 * Loader and container for BVH data.
 * @author kry
 */
public class BVHData {

    /**
     * Name of the loaded BVH file
     */
    public String name = "";
    
    /**
     * Root node of the Skeleton hierarchy
     */
    public SkeletonNode root = null;

    /**
     * List of skeleton nodes in the hierarchy.
     * The order of this list is important for interpreting channel data.
     */
    public ArrayList<SkeletonNode> snodeList = new ArrayList<SkeletonNode>();
    // private ArrayList<SkeletonNode> snodeList = new ArrayList<SkeletonNode>();

    /**
     * Number of frames in the BVH file
     */
    private int frames = 0;
    
    /**
     * Number of channels in the BVH file
     */
    private int channels = 0;
    
    /**
     * Motion data indexed by frame, then by channel, where the channels are
     * in the order of the channel list for each node, going through the nodeList
     * in order.
     */
    private double[][] motionData;

    /**
     * @return the number of frames in the BVH file
     */
    public int getNumFrames() {
        return frames;
    }
    
    /**
     * Converts degrees to radians
     * @param d degrees
     * @return radians
     */
    private double deg2rad( double d ) {
        double r = d * Math.PI / 180;
        while ( r >  Math.PI ) r -= 2*Math.PI;
        while ( r < -Math.PI ) r += 2*Math.PI;
        return r;
    }
    
    /**
     * Converts radians to degrees
     * @param r angle in radians
     * @return angle in degrees
     */
    private double rad2deg( double r ) {
        return r / Math.PI * 180;
    }
    
    /**
     * Sets the skeleton hierarchy parameters with the bvh data
     * at the specified frame number. 
     * 
     * Note: if you have loading multiple BVH files, you might want to
     * make this work with a different provided SkeletonNode hierarchy
     * (i.e., pass in a different root, or perhaps more likely a 
     * different snodeList).
     *  
     * @param fnumber
     */
    public void setSkeletonPose( int fnumber ) {        
        double[] data = motionData[fnumber];        
        int chan = 0;
        for ( SkeletonNode sn : snodeList ) {
            if ( sn.chanlist == null ) continue;
            for ( String chanName : sn.chanlist ) {
                double x = data[chan];
                chan++;
                if ( chanName.equals("Xposition") ) {
                    sn.xTrans.setValue( x );
                } else if ( chanName.equals("Yposition") ) {
                    sn.yTrans.setValue( x );
                } else if ( chanName.equals("Zposition") ) {
                    sn.zTrans.setValue( x );
                } else if ( chanName.equals("Xrotation") ) {
                    sn.xRot.setValue( deg2rad(x) );
                } else if ( chanName.equals("Yrotation") ) {
                    sn.yRot.setValue( deg2rad(x) );
                } else if ( chanName.equals("Zrotation") ) {
                    sn.zRot.setValue( deg2rad(x) );
                }
            }                    
        }
    }
    
    /**
     * An array list for storing a recorded trajectory
     */
    private ArrayList<double[]> trajectory = new ArrayList<double[]>();
    
    /**
     * Clears the trajectory recorded so far
     */
    public void clearSavedTrajectory() {
        trajectory.clear();
    }
    
    /**
     * @return the number of frames in the currently recorded trajectory.
     */
    public int getNumSavedFrames() {
        return trajectory.size();
    }
    
    /**
     * Saves the current pose of the skeleton to the end of the recorded trajectory.
     */
    public void addCurrentPoseToTrajectory() {
        double[] data = new double[channels];
        int chan = 0;
        for ( SkeletonNode sn : snodeList ) {
            if ( sn.chanlist == null ) continue;
            for ( String chanName : sn.chanlist ) {
                if ( chanName.equals("Xposition") ) {
                    data[chan++] = sn.xTrans.getValue();
                } else if ( chanName.equals("Yposition") ) {
                    data[chan++] = sn.yTrans.getValue();
                } else if ( chanName.equals("Zposition") ) {
                    data[chan++] = sn.zTrans.getValue();
                } else if ( chanName.equals("Xrotation") ) {
                    data[chan++] = rad2deg( sn.xRot.getValue() );
                } else if ( chanName.equals("Yrotation") ) {
                    data[chan++] = rad2deg( sn.yRot.getValue() );
                } else if ( chanName.equals("Zrotation") ) {
                    data[chan++] = rad2deg( sn.zRot.getValue() );
                }
            }                    
        }
        trajectory.add(data);
    }
    
    /**
     * Saves the currently recorded trajectory to the file with specified name
     * @param fname
     */
    public void save( String fname ) {
        try {
            PrintStream ps = new PrintStream( new FileOutputStream( fname ) );
            
            ps.println("HIERARCHY");            
            hierarchySaveHelper( "", root, ps );
            
            ps.println("MOTION");
            ps.println("Frames:    " + trajectory.size() );
            // GROSS: we should keep track of the rate that was read below, but
            // for now we'll just leave it as 1/100 s as this is the rate of all
            // our naturalpoint motion capture data
            ps.println("Frame Time:    0.010000");
            
            for ( double[] data : trajectory ) {
                for ( double d : data ) {
                    ps.print( d + " " );
                }
                ps.println();
            }
            
            ps.close();
        } catch ( Exception e ) {
            e.printStackTrace();
            System.err.println("problems writing bvh to file " + fname );
        }
    }
    
    /** 
     * Writes out a node and does a recursive call for the children.
     * 
     * @param indent
     * @param n
     * @param ps
     * @throws IOException
     */
    private void hierarchySaveHelper( String indent, SkeletonNode n, PrintStream ps ) throws IOException {           
        if ( n.children.size() == 0 ) {
            ps.println(indent + "End Site");
            ps.println(indent + "{");
            ps.println(indent + "OFFSET " + n.offset.x + " " + n.offset.y + " " + n.offset.z );
            ps.println(indent + "}");
        } else {
            ps.println(indent + (n.parent == null ? "ROOT " : "JOINT ") + n.name);
            ps.println(indent + "{");
            ps.println(indent + "OFFSET " + n.offset.x + " " + n.offset.y + " " + n.offset.z );
            ps.print(indent + "CHANNELS " + n.chanlist.size() );
            for ( String chan : n.chanlist ) {
                ps.print( " " + chan );
            }
            ps.println();
            String indent2 = indent + "    ";
            for ( SkeletonNode c : n.children ) {
                hierarchySaveHelper(indent2, c, ps);
            }
            ps.println(indent + "}");
        }        
    }
    
    /**
     * Loads the specified BVH file and creates the corresponding SkeletonNode
     * hierarchy.
     * 
     * @param fname
     */
    public void load(String fname) {
        this.name = fname;
        BufferedReader br = null;
        FileReader fr = null;
        try {
            System.out.println( "Loading: " + fname );
            fr = new FileReader( new File( fname ) );
            br = new BufferedReader(fr);

            String n = ""; // name of the node to be created         
            SkeletonNode currentNode = null;  

            String line = br.readLine();
            while ( line != null ) {
                Scanner scan = new Scanner( line );
                String nextUp = scan.next();
                if (line.equals("HIERARCHY")) {
                    // ignore this line
                } else if (nextUp.equals("{")) {
                    SkeletonNode child = new SkeletonNode( n );
                    snodeList.add( child ); 
                    if ( currentNode != null ) currentNode.addChild( child );
                    currentNode = child;
                } else if (nextUp.equals("}")) {
                    currentNode = currentNode.parent;
                } else if (nextUp.equals("OFFSET")) {
                    currentNode.offset.set( scan.nextDouble(), scan.nextDouble(), scan.nextDouble() );
                } else if (nextUp.equals("End")) {
                    n = "End Site";
                } else if (nextUp.equals("CHANNELS")) {
                    int numChannels = scan.nextInt();
                    channels += numChannels;
                    currentNode.chanlist = new ArrayList<String>();
                    for ( int i = 0; i < numChannels; i++ ) {
                        currentNode.chanlist.add( scan.next() );                        
                    }
                } else if (nextUp.equals("MOTION")) {
                    readMotionData( br );
                } else {
                    n = scan.next();
                }
                line = br.readLine();
            }
        } catch ( Exception e ) {
            System.err.println("problems loading file " + fname );
            e.printStackTrace();
        } finally {
            try {
                if ( br != null ) br.close();
                if ( fr != null ) fr.close();
            } catch ( Exception e ) {
                e.printStackTrace();
            }
        }

        // note that the root is always the first child that was created
        root = snodeList.get(0);
        // System.out.println(root);
    }

    private void readMotionData( BufferedReader br ) throws IOException {
        String line = br.readLine();
        Scanner scan = new Scanner(line);
        String nextUp = scan.next();
        if (nextUp.equals("Frames:")) {
            frames = scan.nextInt() + 1;                        
            // pick up all motion data into the matrix to be used
            // later as opposed to live data parsing
            motionData = new double[frames][channels];
            // There will be a line such as "Frame Time:    0.010000"
            // We'll ignore this for now, but come back and reuse it if needed.
            br.readLine(); 
            // Read all motion data lines in this loop to store in
            // the motionData matrix.
            // i is the number of frames, and j is the number of
            // channels in each frame
            line = br.readLine();
            for (int i = 0; line != null; i++) {
                scan = new Scanner(line);
                int j = 0;
                while (scan.hasNext()) {
                    double val;
                    try {
                        val = scan.nextDouble();
                        motionData[i][j] = val;
                    } catch (Exception e) {
                        System.err.println("problems reading frame/channel = " + i + " " + j);
                        throw new RuntimeException("avenge my death!");
                    }
                    j++;
                }
                line = br.readLine();
            }
            line = null;
        }
    }
    
}
