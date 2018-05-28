package odemocap;

import java.util.ArrayList;
import java.util.List;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ode4j.math.DVector3;

import com.jogamp.opengl.util.gl2.GLUT;

import mintools.swing.CollapsiblePanel;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.FancyAxis;
import mintools.viewer.FlatMatrix4d;
import mintools.viewer.SceneGraphNode;

/**
 * A node in a hierarchy or an articulated character.  Each node represents 
 * a joint which rotates about a point which is offset from the parent node's frame. 
 * @author kry
 */
public class SkeletonNode implements SceneGraphNode {

    /**
     * The name of this node 
     */
    public String name;
    
    /**
     * The index of this node to identify it in an external array of all nodes 
     */
    public int index;
    
    /**
     * The parent of this node, or null if it is the root 
     */
    public SkeletonNode parent = null;
    
    /** 
     * List of children nodes 
     */
    public List<SkeletonNode> children = new ArrayList<SkeletonNode>();
    
    /**
     * Offset of this node with respect to parent
     */
    public Point3d offset = new Point3d();
    
    /**
     * List of channels from the BVH file.  This is used to define the order
     * in which this skeleton node applies its transformations.
     */
    public ArrayList<String> chanlist;
    
    /** x translation */
    public DoubleParameter xTrans = new DoubleParameter( "x", 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY );
    /** y translation */
    public DoubleParameter yTrans = new DoubleParameter( "y", 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY );
    /** z translation */
    public DoubleParameter zTrans = new DoubleParameter( "z", 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY );
    /** x Rotation */
    public DoubleParameter xRot = new DoubleParameter( "xRot", 0, -3.14, 3.14 );
    /** y Rotation */
    public DoubleParameter yRot = new DoubleParameter( "yRot", 0, -3.14, 3.14 );
    /** z Rotation */
    public DoubleParameter zRot = new DoubleParameter( "zRot", 0, -3.14, 3.14 );
    /** reference to the ode body that represents this skeleton node */
    public BoxBodyODE odeBody;
    
    /**
     * Creates a new skeleton node with no parent and the given name.
     * The offset defaults to zero, and the default rotation order without
     * a channel list from a BVH file is z y x.
     * @param name
     */
    public SkeletonNode( String name ) {        
        this.name = name;
    }
        
    /**
     * Adds a child node to this node and sets this node as 
     * the child's parent.
     * @param n
     */
    public void addChild( SkeletonNode n ) {
        n.parent = this;
        children.add( n );
    }
        
    /** 
     * Position of the bounding volume geometry that represents this skeleton node 
     */
    public Point3d geometryPosition = new Point3d();
    
    /**
     * Size in each axis direction of the bounding volume geometry that represents this skeleton node  
     */
    public Vector3d geometrySize = new Vector3d();

    /**
     * Finds bounding geometry for this node based on its
     * location and the location of its children, then 
     * calls init on children.
     */
    @Override
    public void init(GLAutoDrawable drawable) {
        // lower left and upper right positions
        Point3d gll = new Point3d();
        Point3d gur = new Point3d();
   
        for ( SkeletonNode s : children ) {
            if ( s.offset.x < gll.x ) gll.x = s.offset.x;
            if ( s.offset.y < gll.y ) gll.y = s.offset.y;
            if ( s.offset.z < gll.z ) gll.z = s.offset.z;
            if ( s.offset.x > gur.x ) gur.x = s.offset.x;
            if ( s.offset.y > gur.y ) gur.y = s.offset.y;
            if ( s.offset.z > gur.z ) gur.z = s.offset.z;
        }
        geometryPosition.add( gll, gur );
        geometryPosition.scale( 0.5 );            
        geometrySize.sub( gur, gll );
        geometrySize.scale( 0.5 );
        
        // make sure that nothing is completely flat!
        double size = axisSize.getValue();
        geometrySize.x = ( geometrySize.x < size ) ? size : geometrySize.x; 
        geometrySize.y = ( geometrySize.y < size ) ? size : geometrySize.y;
        geometrySize.z = ( geometrySize.z < size ) ? size : geometrySize.z;
        
        if (name.endsWith("Ankle"))
        		geometrySize.z *= 1.5;
        
        geometrySize.x *= 0.95;

        for ( SkeletonNode c : children ) {
            c.init( drawable );
        }
    }
    
    /**
     * Geometry for an axis for showing the orientation of each joint
     */
    private static FancyAxis fa = new FancyAxis();
    
    /**
     * Parameter for scaling the axes shown at the joints and the geometry around each bone.
     * This is made public so that the size can be easily set when loading different data that
     * was recorded with different units (e.g., Vicon data from the CMU database, or our own 
     * natural point motion capture).
     */
    public static DoubleParameter axisSize = new DoubleParameter( "axis size", 5, 0.01, 100 );
    
    private static BooleanParameter drawLines = new BooleanParameter( "draw skeleton lines", false );
    private static BooleanParameter drawAxes= new BooleanParameter( "draw joint axes", false );
    private static BooleanParameter drawGeometry = new BooleanParameter( "draw skeleton geometry", true );
    private static BooleanParameter drawNames = new BooleanParameter( "draw skeleton names", false );

    /**
     * Gets the control panel for adjusting the display of all skeleton nodes
     * @return a control panel
     */
    public static JPanel getStaticControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder("Skeleton drawing controls" ) );
        vfp.add( drawAxes.getControls() );
        vfp.add( drawLines.getControls() );
        vfp.add( drawGeometry.getControls() );
        vfp.add( drawNames.getControls() );
        vfp.add( axisSize.getSliderControls(true) );
        return vfp.getPanel();
    }
    
    
    @Override
    public void display(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();
        
        // can draw a line segment as part of the skeleton
        if ( drawLines.getValue() ) {
            gl.glDisable( GL2.GL_LIGHTING );
            gl.glColor4d( 1, 1, 1, 0.5 );
            gl.glLineWidth(3);
            gl.glBegin(GL.GL_LINES);
            gl.glVertex3d(0,0,0);
            gl.glVertex3d(offset.x, offset.y, offset.z);
            gl.glEnd();
            gl.glEnable( GL2.GL_LIGHTING );
        }
        
        // get the transform that brings us to the location of 
        // this joint with respect to its parent, and apply
        gl.glPushMatrix();
        getTransform( T.getBackingMatrix() );
        gl.glMultMatrixd( T.asArray(), 0 );
                
        // We might want to draw the geometry for the meat around this part of 
        // the skeleton, such as a scaled sphere to match the shape of the body 
        // part. 
        
        if ( drawGeometry.getValue() && children.size() != 0 ) {
            float[] frontColour = { .6f, .4f, .4f, 1};
            float[] specColour  = { .8f, .8f, .8f, 1};
            float[] zeroColour  = { 0, 0, 0, 1};
            gl.glMaterialfv( GL.GL_FRONT, GL2.GL_AMBIENT, zeroColour, 0 );
            gl.glMaterialfv( GL.GL_FRONT, GL2.GL_DIFFUSE, frontColour, 0 );
            gl.glMaterialfv( GL.GL_FRONT, GL2.GL_SPECULAR, specColour, 0 );
            gl.glColor3fv( frontColour, 0 );
            gl.glMateriali( GL.GL_FRONT, GL2.GL_SHININESS, 92 );
            gl.glPushMatrix();
            gl.glTranslated( geometryPosition.x, geometryPosition.y, geometryPosition.z );
            // use the size of the bounding box to draw the geometry, but 
            // make sure that each dimension is at least a minimum specified size.
            tmpVector.set( geometrySize );
            double size = axisSize.getValue();
            tmpVector.x = ( geometrySize.x < size ) ? size : geometrySize.x; 
            tmpVector.y = ( geometrySize.y < size ) ? size : geometrySize.y;
            tmpVector.z = ( geometrySize.z < size ) ? size : geometrySize.z;
            gl.glScaled( tmpVector.x, tmpVector.y, tmpVector.z );
            EasyViewer.glut.glutSolidSphere( 1, 32, 16 );
            //EasyViewer.glut.glutSolidCube( 2 );
            gl.glPopMatrix();
        }
        
        if ( drawAxes.getValue() ) {
            fa.setSize( axisSize.getValue() * 4 );
            fa.draw(gl);
        }
        if ( drawNames.getValue() ) {
            gl.glDisable( GL2.GL_LIGHTING );
            gl.glColor4f(1,1,1,1);
            gl.glRasterPos3d( 0,0,0 );            
            EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, name);
            gl.glEnable( GL2.GL_LIGHTING );
        }
        
        for ( SkeletonNode c : children ) {
            c.display(drawable);
        }
        
        gl.glPopMatrix();
    }
    
    private FlatMatrix4d T = new FlatMatrix4d();
    private Matrix4d tmpMatrix = new Matrix4d();
    private Matrix4d tmpMatrix2 = new Matrix4d();
    private Vector3d tmpVector = new Vector3d();

    /**
     * Gets the transformation between this node and its parent.
     * 
     * Note, this method deals with translation in a way that might be cheating
     * slightly. There is nothing to say that the translation channels can be
     * mixed with the rotation channels (i.e., they don't appear all at the
     * beginning of the channel list). But in all examples I've seen, these
     * translations come first, so this will very likely work for just about any
     * bvh file.
     * 
     * @param m
     */
    public void getTransform( Matrix4d m ) {
        tmpVector.set( xTrans.getValue(), yTrans.getValue(), zTrans.getValue() );
        tmpVector.add( offset );
        // set the translation part of the matrix
        m.set( tmpVector );
        // now compute and set the rotation part of the matrix.
        if ( chanlist != null ) {
            for( String chan : chanlist ) {
                if ( chan.equals("Xrotation") ) {
                    tmpMatrix.rotX(xRot.getValue());
                    m.mul(tmpMatrix);
                } else if ( chan.equals("Yrotation") ) {
                    tmpMatrix.rotY(yRot.getValue());
                    m.mul(tmpMatrix);
                } else if ( chan.equals("Zrotation") ) {
                    tmpMatrix.rotZ(zRot.getValue());
                    m.mul(tmpMatrix);
                }
            }
        } else {
            // just use a default order if there is no channel list
            tmpMatrix.rotX(zRot.getValue());
            m.mul(tmpMatrix);
            tmpMatrix.rotY(xRot.getValue());
            m.mul(tmpMatrix);
            tmpMatrix.rotZ(yRot.getValue());
            m.mul(tmpMatrix);
        }
    }
    
    /**
     * Frames from the previous frame which can be used to compute world velocity of this body.
     */
    public Matrix4d EwfrombPrevious = new Matrix4d();
    
    /** linear part of the twist produced by this joint's velocity, in world coordinates */
    public Vector3d worldVelocity = new Vector3d();
    
    /** angular part of the twist produced by this joint's velocity, in world coordinates */
    public Vector3d worldOmega = new Vector3d();
    
    /**
     * Creates a snapshot of the current pose, to be compared with another pose to compute the velocity of bodies
     */
    public void copyEwfrombToPrevious() {
        EwfrombPrevious.set( Ewfromb.getBackingMatrix() );
        for( SkeletonNode n : children ) {
            n.copyEwfrombToPrevious();
        }
    }
    
    /**
     * World linear velocity of the body.
     * Note that the program flow to compute this is a bit awkward and could be reorganized.
     * The process is:  
     * 1) set the skeleton from the bvh
     * 2) compute transforms
     * 3) save a copy with copyEwfrombToPrevious
     * 4) set the skeleton from the bvh for the next frame in the animation
     * 5) compute transforms
     * 6) call computeWorldTwist on each node in the skeleton with the stepsize between frames.  
     */
    public DVector3 v = new DVector3();
    /** 
     * World angular velocity of the body.  See comment above on computation. 
     */
    public DVector3 w = new DVector3();
    
    /**
     * Computes the world coordinate twist caused by this joint using 
     * the previous and current transform and the given time step.
     * @param h 
     */
    public void computeWorldTwist( double h ) {
        Matrix4d Ewfbpinv = new Matrix4d();
        Matrix4d tmp = new Matrix4d();
        Ewfbpinv.set( EwfrombPrevious );
        Ewfbpinv.invert();
        tmp.mul( Ewfromb.getBackingMatrix(), Ewfbpinv );
        // might want to check that tmp looks like a homogeneous velocity once the identity is subtracted
        // could actually do a log of the rotation to find the true angular component, but we can probably
        // safely assume that the motions are small enough for this to work and be accurate.
        w.set( -tmp.m12,  tmp.m02, -tmp.m01 );
        v.set(  tmp.m03,  tmp.m13,  tmp.m23 );
        w.scale( 1 / h );
        v.scale( 1 / h );
    }
    
    /**
     * Transformation from the body frame to the world frame
     */
    public FlatMatrix4d EwfrompT = new FlatMatrix4d(); 
    /**
     * Cumulative transforms for each of the rotations
     */
    public FlatMatrix4d EwfrompTR[] = null;
    /**
     * Total transformation including both translation and rotation
     */
    public FlatMatrix4d Ewfromb = null;
    
    private Matrix4d R[] = { new Matrix4d(), new Matrix4d(), new Matrix4d() };
    
    /**
     * The axis associated with each of the rotations
     */
    public Vector3d axis[] = { new Vector3d(), new Vector3d(), new Vector3d() };
    
    /**
     * Computes all the transforms of all the body parts
     */
    public void computeTransforms( ) {
        Matrix4d I = new Matrix4d();
        I.setIdentity();
        computeTransformHelper( I );
    }
    
    private void computeTransformHelper( Matrix4d m ) {
        
        if ( EwfrompTR == null ) {
            EwfrompTR = new FlatMatrix4d[3];        
            for ( int i = 0; i < 3; i++ ) {
                EwfrompTR[i] = new FlatMatrix4d();
            }      
            Ewfromb = EwfrompTR[2];
        }
        
        tmpVector.set( xTrans.getValue(), yTrans.getValue(), zTrans.getValue() );
        tmpVector.add( offset );
        tmpMatrix2.set( tmpVector ); // set the matrix to a translation matrix
                
        EwfrompT.getBackingMatrix().mul( m, tmpMatrix2 );
        
        if ( chanlist != null ) {
            int i = 0;
            for( String chan : chanlist ) {
                if ( chan.equals("Xrotation") ) {
                    axis[i].set(1,0,0);
                    R[i++].rotX(xRot.getValue());                        
                } else if ( chan.equals("Yrotation") ) {
                    axis[i].set(0,1,0);
                    R[i++].rotY(yRot.getValue());                    
                } else if ( chan.equals("Zrotation") ) {
                    axis[i].set(0,0,1);
                    R[i++].rotZ(zRot.getValue());
                }
            }
        } else {
            // just use a default order if there is no channel list
            axis[0].set(1,0,0);
            R[0].rotX(zRot.getValue());
            axis[1].set(0,1,0);
            R[1].rotY(xRot.getValue());
            axis[2].set(0,0,1);
            R[2].rotZ(yRot.getValue());            
        }
        
        EwfrompTR[0].getBackingMatrix().mul(EwfrompT.getBackingMatrix(), R[0] );
        EwfrompTR[1].getBackingMatrix().mul(EwfrompTR[0].getBackingMatrix(), R[1] );
        EwfrompTR[2].getBackingMatrix().mul(EwfrompTR[1].getBackingMatrix(), R[2] );
        
        for ( SkeletonNode c : children ) {
            c.computeTransformHelper(EwfrompTR[2].getBackingMatrix());
        }
    }
    
    /** 
     * Adds a rotation to the currently set rotations.
     * @param r rotation to add in radians
     */
    public void addRotation( double[] r ) {
        int i = 0;
        if ( chanlist != null ) {
            for( String chan : chanlist ) {
                if ( chan.equals("Xrotation") ) {
                    double angle = xRot.getValue() + r[i++]; 
                    while ( angle > Math.PI ) angle-= Math.PI*2;
                    while ( angle < -Math.PI ) angle+= Math.PI*2;
                    xRot.setValue(angle);
                } else if ( chan.equals("Yrotation") ) {
                    double angle = yRot.getValue() + r[i++]; 
                    while ( angle > Math.PI ) angle-= Math.PI*2;
                    while ( angle < -Math.PI ) angle+= Math.PI*2;
                    yRot.setValue(angle);         
                } else if ( chan.equals("Zrotation") ) {
                    double angle = zRot.getValue() + r[i++]; 
                    while ( angle > Math.PI ) angle-= Math.PI*2;
                    while ( angle < -Math.PI ) angle+= Math.PI*2;
                    zRot.setValue(angle);
                }
            }
        } else {
            // Note: chanlist should never be null!
            System.err.println("chanlist is null!");
            i = 0;
            double angle;
            angle = zRot.getValue() + r[i++]; 
            while ( angle > Math.PI ) angle-= Math.PI*2;
            while ( angle < -Math.PI ) angle+= Math.PI*2;
            zRot.setValue(angle);
            angle = xRot.getValue() + r[i++]; 
            while ( angle > Math.PI ) angle-= Math.PI*2;
            while ( angle < -Math.PI ) angle+= Math.PI*2;
            xRot.setValue(angle);
            angle = yRot.getValue() + r[i++]; 
            while ( angle > Math.PI ) angle-= Math.PI*2;
            while ( angle < -Math.PI ) angle+= Math.PI*2;
            yRot.setValue(angle);         
            
        }
    }
    
    /**
     * Resets the skeleton hierarchy to its zero pose.
     * Note that this is often the same as the T pose.
     */
    public void resetToZeroPose() {
        xTrans.setValue( 0 );
        yTrans.setValue( 0 );
        zTrans.setValue( 0 );
        zRot.setValue( 0 );
        xRot.setValue( 0 );
        yRot.setValue( 0 );
        for ( SkeletonNode n : children ) {
            n.resetToZeroPose();
        }
    }
        
    @Override   
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.setBorder( new TitledBorder( name ) );
        VerticalFlowPanel vfp2 = new VerticalFlowPanel();
        vfp2.setBorder( new TitledBorder ("parameters"));
        vfp2.add( xTrans.getControls() );
        vfp2.add( yTrans.getControls() );
        vfp2.add( zTrans.getControls() );        
        vfp2.add( xRot.getSliderControls(false) );
        vfp2.add( yRot.getSliderControls(false) );
        vfp2.add( zRot.getSliderControls(false) );
        CollapsiblePanel cp = new CollapsiblePanel( vfp2.getPanel() );
        cp.collapse();
        vfp.add( cp );
        for ( SkeletonNode c : children ) {
            vfp.add( c.getControls() );
        }        
        return vfp.getPanel();
    }    
    
    @Override
    public String toString() {     
        return toStringWithPrefix("");
    }

    /**
     * Builds a nicely indented string description of the skeleton's structure
     * @param prefix
     * @return a string describing the skeleton
     */
    private String toStringWithPrefix( String prefix ) {
        String s = prefix + name + "\n";
        for ( SkeletonNode c : children ) {
            s += c.toStringWithPrefix( prefix + "  " );
        }
        return s;
    }
}
