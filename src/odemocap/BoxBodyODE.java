package odemocap;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import mintools.parameters.BooleanParameter;
import mintools.viewer.EasyViewer;
import mintools.viewer.FlatMatrix4d;

import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

import com.jogamp.opengl.util.gl2.GLUT;


/**
 * Implementation of some opengl glue and other utilities for creating body parts that
 * can be simulated with ODE
 * @author kry
 */
public class BoxBodyODE {

    /** the name of this body */
    public String name;
    
    /** the ODE body object */
    public DBody body;
    /** the mass of the body */
    public DMass mass;
    /** the geometry of the body */
    public DBox box;
    
    private double dimx = 1;
    private double dimy = 1;
    private double dimz = 1;
    private DVector3 p0 = new DVector3();
    private DQuaternion q0 = new DQuaternion(1,0,0,0);
    
    /**
     * parameter to enable visual display of the name of boxbody objects
     */
    public static BooleanParameter displayName = new BooleanParameter( "display body names", false );
    
    /**
     * 
     * @param name
     * @param world 
     * @param space
     * @param dimx
     * @param dimy
     * @param dimz
     * @param position
     * @param orientation
     */
    public BoxBodyODE( String name, DWorld world, DSpace space, double dimx, double dimy, double dimz, DVector3 position, DQuaternion orientation ) {
        this.name = name;
        this.dimx = dimx;
        this.dimy = dimy;
        this.dimz = dimz;
        p0.set( position );
        q0.set( orientation );
        
        body = OdeHelper.createBody( world );
        mass = OdeHelper.createMass();
        mass.setBox( 1, dimx, dimy, dimz ); 
        // TODO: density 1000 or 1 ???  not clear!  perhaps that's why people call adjustMass (i.e., to scale)
        body.setMass( mass);
        box = OdeHelper.createBox( space, dimx, dimy, dimz );
        box.setBody( body );        
    }
    
    private FlatMatrix4d M = new FlatMatrix4d();

    /**
     * Displays the body in its current position
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        
        gl.glColor4f(0,0.7f,0,1);
        gl.glPushMatrix();        
        
        ODESimulation.setFlatMatrix( body, M );
        gl.glMultMatrixd( M.asArray(), 0 );
        gl.glScaled( dimx, dimy, dimz );
        EasyViewer.glut.glutSolidCube(1);
        
        if ( displayName.getValue() ) {
            gl.glDisable( GL2.GL_LIGHTING );
            gl.glColor4f(1,1,1,0.8f);
            gl.glBegin( GL.GL_LINES );
            gl.glVertex3d(0,0,0);
            gl.glVertex3d(1.1,1.1,1.1);
            gl.glEnd();
            gl.glRasterPos3d(1.1,1.1,1.1);
            EasyViewer.glut.glutBitmapString(GLUT.BITMAP_8_BY_13, name);
            gl.glEnable( GL2.GL_LIGHTING );
        }
        
        gl.glPopMatrix();
    }

    /**
     * Sets the configuration of this body to the given homogeneous transformation matrix
     * @param EwfromRB
     * @param scale 
     */
    public void setPosition( Matrix4d EwfromRB, double scale ) {
        Matrix3d R = new  Matrix3d();
        Vector3d p = new Vector3d();
        EwfromRB.getRotationScale(R);
        EwfromRB.get(p);
        p.scale( scale );
        Quat4d q = new Quat4d();
        q.set(R);
        if ( q.w < 0 ) {
            System.out.println(" watch out!" );
        }
        DVector3 ptmp = new DVector3( p.x, p.y, p.z );
        DQuaternion qtmp = new DQuaternion(  q.w, q.x, q.y, q.z ); // note real comes first in a DQuaternion?? 
        body.setPosition( ptmp );
        body.setQuaternion( qtmp );
        body.setLinearVel( 0,0,0 );
        body.setAngularVel( 0,0,0 );
    }
    
    /**
     * Sets the velocity of this body, given the body's velocity in world coordinates
     * @param vel
     * @param omega
     * @param scale  scale to convert from the units in the BVH to the units in ODE
     * @param extra  extra scaling parameter
     */
    public void setVelocity( DVector3 vel, DVector3 omega, double scale, double extra ) {        
        // argh, yes, these should take the body velocity in world aligned body coordinates.
        // poorly documented, but probably the case!
        DVector3 pos = new DVector3( body.getPosition() );
        pos.scale( 1 / scale );
        DVector3 tmp = new DVector3();
        DVector3 v = new DVector3( vel);
        DVector3 w = new DVector3( omega );
        tmp.eqCross( w, pos );
        v.add( tmp );
        v.scale( scale );
        v.scale( extra );
        w.scale( extra );
        body.setLinearVel( v );
        body.setAngularVel( w );
    }
    
    /**
     * Resets the position of this body to its initial configuration
     */
    public void reset() {
        body.setPosition( p0 );
        body.setQuaternion( q0 );
        body.setLinearVel( 0,0,0 );
        body.setAngularVel( 0,0,0 );
    }
    
}
