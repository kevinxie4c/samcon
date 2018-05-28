package odemocap;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import javax.swing.JPanel;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.FancyArrow;
import mintools.viewer.FlatMatrix4d;

import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAMotorJoint;
import org.ode4j.ode.DBallJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;

/**
 * Simple example of how to get contact forces
 * @author kry
 */
public class ODESimulation {

    private DWorld world;
    private DSpace space;
    private DJointGroup contactgroup;
    private DJointGroup skeletonJointGroup; 
    
    private Set<DBody> nonCollidable = new HashSet<DBody>();

    /*
    private DBody sphereBody;
    private DMass sphereMass;
    private DSphere sphere;
    */

    private DBody boxBody;
    private DMass boxMass;
    private DBox box;
    
    private List<BoxBodyODE> bodies = new ArrayList<BoxBodyODE>();
    
    private BooleanParameter animate = new BooleanParameter( "animate", false );   
    private BooleanParameter wireframe = new BooleanParameter( "wireframe", false );        
    private DoubleParameter scaleForceDisplay = new DoubleParameter("scale force display", 0.1, 1e-3, 1e3 );
    private BooleanParameter drawContacts = new BooleanParameter( "draw contacts", false );  
    /**
     * Careful with this parameter... should be joint specific.  Currently as high as 1 and low as 0.1 are 
     * reasonable.  Should be inertia weighted.  Should double check inertia settings of bodies due to 
     * unclear documentation with respect to density parameter.
     */
    private DoubleParameter stiffnessParam = new DoubleParameter( "ball joint stiffness", 0.8, 1e-3, 1e3 );
    private DoubleParameter dampingParam = new DoubleParameter( "ball joint damping", 0.01, 1e-3, 1e3 );   // current stiffness and damping comes from questionable masses?? 
    //private DoubleParameter stiffnessParam = new DoubleParameter( "ball joint stiffness", 2, 1e-3, 1e3 );
    //private DoubleParameter dampingParam = new DoubleParameter( "ball joint damping", 0.01, 1e-3, 1e3 );   // current stiffness and damping comes from questionable masses??    
    private BooleanParameter applyStiffness = new BooleanParameter( "apply stiffness torques", true );
    private BooleanParameter applyDamping = new BooleanParameter( "apply damping torques", true );
    
    private DoubleParameter stepSize = new DoubleParameter( "step size", 0.01, 1e-4, 0.1 );    
    private IntParameter subSteps = new IntParameter( "substeps", 100, 1, 1000 );
    private BooleanParameter enableGravity = new BooleanParameter( "enable gravity", true );
    private DoubleParameter gravity = new DoubleParameter( "gravity", -9.8, -9.8, 9.8 );
    private DoubleParameter contactCFM = new DoubleParameter( "contact cfm", 0, 0, 1000 );
    private DoubleParameter contactERP = new DoubleParameter( "contact erp", 0.2, 0.1, 0.8 );
    private DoubleParameter worldCFM = new DoubleParameter( "world cfm", 0, 0, 1000 );
    private DoubleParameter worldERP = new DoubleParameter( "world erp", 0.2, 0.1, 1000 );
    private BooleanParameter enableContacts = new BooleanParameter( "enable contacts", true );
    private DoubleParameter friction = new DoubleParameter( "friction coefficient", 0.33, 0, 0.5 );
    //private DoubleParameter friction = new DoubleParameter( "friction coefficient", 0.5, 0, 0.5 );
    private DoubleParameter extraVelocityScale = new DoubleParameter( "extra velocity scale (TEST)", 1, -1, 1 );
    
    /**
     * @return a control panel for adjusting simulation and viewing parameters
     */
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        vfp.add( scaleForceDisplay.getSliderControls(true) );
        vfp.add( animate.getControls() );
        vfp.add( wireframe.getControls() );
        vfp.add( stiffnessParam.getSliderControls(true) );
        vfp.add( dampingParam.getSliderControls(true) );        
        vfp.add( applyStiffness.getControls() );
        vfp.add( applyDamping.getControls() );
        vfp.add( subSteps.getSliderControls() );
        vfp.add( stepSize.getSliderControls(true) );
        vfp.add( BoxBodyODE.displayName.getControls() );
        vfp.add( drawContacts.getControls() );
        vfp.add( enableGravity.getControls() );
        vfp.add( gravity.getSliderControls(false));
        vfp.add( contactCFM.getSliderControls(false));
        vfp.add( contactERP.getSliderControls(false));
        vfp.add( worldCFM.getSliderControls(false));
        vfp.add( worldERP.getSliderControls(false));
        vfp.add( extraVelocityScale.getSliderControls(false));
        vfp.add( enableContacts.getControls() );
        vfp.add( friction.getControls() );
        return vfp.getPanel();        
    }
    
    /**
     * An amount to lower the floor so that the feet of the character defined in the bvh 
     * are not below the floor to start
     */
    //public double floorOffset = 0.06;//-0.025;
    public double floorOffset = -0.01;
    
    /**
     * Creates the world for ODE and initializes objects.
     */
    public ODESimulation() {
        
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        space = OdeHelper.createHashSpace(null);
        contactgroup = OdeHelper.createJointGroup ();
        skeletonJointGroup = OdeHelper.createJointGroup();
        // create a floor and four walls
        OdeHelper.createPlane (space, 0, 1, 0,  floorOffset);
        OdeHelper.createPlane (space, 1, 0, 0, -4);
        OdeHelper.createPlane (space,-1, 0, 0, -4);
        OdeHelper.createPlane (space, 0, 0,-1, -4);
        OdeHelper.createPlane (space, 0, 0, 1, -4);
        
        initObjects();
    }

    /**
     * Cleans up ODE objects.  Call this if you wish to discard this ODESimulation.
     */
    public void cleanup() {
        contactgroup.destroy();
        space.destroy();
        world.destroy();
        OdeHelper.closeODE();
    }

        
    private double scale; // scale for the skeleton node into the world
    
    private SkeletonNode root;
    
    /**
     * Creates a skeleton from the given hierarchy root node.
     * Translations are scaled by the given amount.
     * @param root
     * @param s
     */
    public void createSkeleton( @SuppressWarnings("hiding") SkeletonNode root, double s ) {
        scale = s;
        this.root = root;
        root.resetToZeroPose();
        
        root.computeTransforms();
        createSkeletonHelper(root );
    }
    
    private void createSkeletonHelper( SkeletonNode node ) {
        if ( node.name.compareTo("End Site") == 0 ) return;
        
        Matrix4d E = node.Ewfromb.getBackingMatrix();
        Matrix3d R = new  Matrix3d();
        Vector3d p0 = new Vector3d();
        E.getRotationScale(R);
        E.get(p0);
        Quat4d q0 = new Quat4d();
        q0.set(R);        
        p0.add( node.geometryPosition );
        p0.scale(scale);
        
        DVector3 dp0 = new DVector3( p0.x, p0.y, p0.z );
        DQuaternion dq0 = new DQuaternion( q0.w, q0.x, q0.y, q0.z );
        
        double dimx = node.geometrySize.x * scale * 2;
        double dimy = node.geometrySize.y * scale * 2;
        double dimz = node.geometrySize.z * scale * 2;        
        BoxBodyODE b = new BoxBodyODE( node.name, world, space, dimx, dimy, dimz, dp0, dq0 );
        node.odeBody = b;
        if (node.name.equals("Head") || node.name.equals("neck") || node.name.endsWith("Collar")
        		|| node.name.endsWith("Shoulder") || node.name.endsWith("Elbow")
        		|| node.name.endsWith("Wrist"))
        		b.mass.setMass(1);
        bodies.add(b);
        for ( SkeletonNode child : node.children ) {
            createSkeletonHelper(child);
        }
        
        if ( node.name.contains("Collar") ) nonCollidable.add(b.body);
        if ( node.name.contains("Chest") ) nonCollidable.add(b.body);
        
    }
    
    /**
     * Creates the skeleton
     * @param node
     */
    public void createSkeletonJoints( SkeletonNode node ) {
        if ( node.name.compareTo("End Site") == 0 ) return;
        if ( node.parent != null ) { // && !node.name.contains("Chest") ) {
            // compute joint anchor position (world coordinates)
            Matrix4d Ewfromb = node.parent.Ewfromb.getBackingMatrix();
            Point3d p = new Point3d();
            Ewfromb.transform( node.offset, p );
            p.scale( scale );
//            if ( false && node.name.contains("Knee") ) {    
//            	DHingeJoint hinge = OdeHelper.createHingeJoint(world, skeletonJointGroup );
//            	DAMotorJoint motor = OdeHelper.createAMotorJoint(world, skeletonJointGroup);
//            	motor.set
//            	//JointHinge hinge = new JointHinge( world, skeletonJointGroup );
//                hinge.attach( node.parent.odeBody.body, node.odeBody.body );   
//                hinge.setAnchor( (float) p.x, (float) p.y, (float) p.z );                
//                // should probably extract this in a useful way?
//                hinge.setAxis1( 1, 0, 0 );       
//                hinge.setMinAngleStop( (float) (-120 * Math.PI/180));
//                hinge.setMaxAngleStop( (float) (10 * Math.PI/180));
//                
//            } else {
                DBallJoint  ball = OdeHelper.createBallJoint(world, skeletonJointGroup);
                ball.attach( node.odeBody.body, node.parent.odeBody.body );            
                ball.setAnchor( p.x, p.y, p.z );                   
//            }
        }
        for ( SkeletonNode child : node.children ) {
            createSkeletonJoints( child );
        }
    }
    
    private static void setQuat4d( Quat4d dst, DQuaternionC src ) {
        dst.w = src.get0();
        dst.x = src.get1();
        dst.y = src.get2();
        dst.z = src.get3();
    }
    
    /**
     * Applies torque to the joints to pull the character towards the 
     * current pose of the skeleton
     * @param node
     */
    public void applyJointTorques( SkeletonNode node ) {
        if ( node.name.compareTo("End Site") == 0 ) return;
        if ( node.parent != null ) {
            if ( applyStiffness.getValue() ) { // && node.name.contains("Hip") ) {
                // find the current desired relative orientation             
                Matrix4d Epfromb = new Matrix4d();
                node.getTransform(Epfromb);
                Matrix3d R = new Matrix3d();
                Epfromb.getRotationScale(R);
                Quat4d qTarget = new Quat4d();
                qTarget.set(R);
                Quat4d qT = new Quat4d();
                qT.set(  qTarget.x,  qTarget.y,  qTarget.z,  qTarget.w );
                // find the current relative orientation
                Quat4d q1 = new Quat4d();
                Quat4d q2 = new Quat4d();
                Quat4d q2inv = new Quat4d();
                setQuat4d( q1, node.parent.odeBody.body.getQuaternion() );
                setQuat4d( q2, node.odeBody.body.getQuaternion() );                
                q2inv.inverse(q2);
                
                Quat4d qdiff = new Quat4d(0,0,0,1);
                qdiff.mul(q2inv);
                qdiff.mul(q1);                
                qdiff.mul(qT);
                if ( qdiff.w < 0 ) { 
                    // does this ever happen?  probably doesn't cause problems even if it does.
                    qdiff.scale(-1);
                }                
                AxisAngle4d aa = new AxisAngle4d();
                aa.set( qdiff );                
                double stiffness = stiffnessParam.getValue();                 
                double angle = aa.angle;
                Vector3d tau = new Vector3d( aa.x, aa.y, aa.z );
                // This axis is in body coordinates, so it is necessary to transform it to the world.
                // note that since the linear force component is zero, it will also be zero in the world frame
                Matrix3d Rq2 = new Matrix3d();
                Rq2.set( q2 );
                Rq2.transform( tau );
                tau.normalize();
                if ( angle > Math.PI ) {
                    angle -= Math.PI * 2;
                    System.out.println(" warning: large angle between target and actual pose" );
                }
                tau.scale( (angle * stiffness) );
                DVector3 dtau = new DVector3( tau.x, tau.y, tau.z );                
                node.odeBody.body.addTorque( dtau );
                dtau.scale( -1 );
                node.parent.odeBody.body.addTorque( dtau );
            }
            if ( applyDamping.getValue() ) {                
                DVector3 omega1 = new DVector3( node.parent.odeBody.body.getAngularVel() );
                DVector3 omega2 = new DVector3( node.odeBody.body.getAngularVel() );
                DVector3 tauf = new DVector3( omega2 );
                tauf.sub( omega1 ); 
                tauf.scale(  dampingParam.getValue() );                
                node.parent.odeBody.body.addTorque( tauf );
                tauf.scale( -1 );
                node.odeBody.body.addTorque( tauf );                
            }
        }
            
        for ( SkeletonNode child : node.children ) {
            applyJointTorques( child );
        }
    }
    
    String text = "";
    
    /**
     * Sets the current pose velocity for the given node and its children
     * @param node
     * @param h
     */
    public void setCurrentPoseVelocity( SkeletonNode node, double h ) {
        // UGH... proper velocity computation must come from comparing frames, NOT from successive joint angles (CLASSIC problem with Euler angles)
        if ( node.name.compareTo("End Site") == 0 ) return;        
        node.computeWorldTwist( h );
        node.odeBody.setVelocity( node.v, node.w, scale, extraVelocityScale.getValue() );
        for ( SkeletonNode child : node.children ) {
            setCurrentPoseVelocity( child, h );
        }        
    }
    
    /**
     * Sets the state of the skeleton rigid bodies to the current position of the skeleton.
     * @param node
     */
    public void setCurrentPose( SkeletonNode node ) {
        if ( node.name.compareTo("End Site") == 0 ) return;
        
        Matrix4d Ewfromb = node.Ewfromb.getBackingMatrix();
        Matrix4d EbfromRB = new Matrix4d();
        Vector3d trans = new Vector3d( node.geometryPosition );
        EbfromRB.set( trans );
        
        Matrix4d EwfromRB = new Matrix4d();
        EwfromRB.mul( Ewfromb, EbfromRB );
        
        node.odeBody.setPosition( EwfromRB, scale );
        
        for ( SkeletonNode child : node.children ) {
            setCurrentPose(child);
        }
    }
    
    /**
     * Displays the objects in the ODE simulation in their current positions
     * @param drawable
     */
    public void display( GLAutoDrawable drawable ) {
        GL2 gl = drawable.getGL().getGL2();
        
        if ( wireframe.getValue() ) {
            gl.glPolygonMode(GL.GL_FRONT_AND_BACK, GL2.GL_LINE);
        }
        
        gl.glEnable( GL2.GL_COLOR_MATERIAL );
        gl.glColorMaterial( GL.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE );

        /*
        gl.glPushMatrix();        
        gl.glColor3f( 0.7f,0,0 );        
        setFlatMatrix( sphereBody, M );
        gl.glMultMatrixd( M.asArray(), 0 );
        // we know it is radius 1
        EasyViewer.glut.glutSolidSphere(1, 64,32);
        gl.glPopMatrix();
        */
        
        gl.glPushMatrix();
        gl.glColor4f(0,0,0.7f,1);
        setFlatMatrix( boxBody, M );
        gl.glMultMatrixd( M.asArray(), 0 );
        gl.glScaled( 0.3, 0.4, 0.6 );
        EasyViewer.glut.glutSolidCube(1);
        gl.glPopMatrix();
        
        gl.glColor3f( 0, 0.7f, 0.7f );

        for ( BoxBodyODE b : bodies ) {
            b.display( drawable);
        }
                
        if ( wireframe.getValue() ) {
            gl.glPolygonMode(GL.GL_FRONT_AND_BACK, GL2.GL_FILL);
        }  
        
        gl.glDisable( GL2.GL_COLOR_MATERIAL );
        
        if ( drawContacts.getValue() ) {
            // draw contact information
//            for ( ContactPair cp : L ) {
//                contact.setIndex(cp.collisionIndex);
//                contact.getPosition(pos);
//                SWIGTYPE_p_float tmpNormal = cp.fb.getF1();
//                float s = (float) scaleForceDisplay.getValue();
//                float nx = Ode.floatArray_getitem( tmpNormal, 0 );
//                float ny = Ode.floatArray_getitem( tmpNormal, 1 );
//                float nz = Ode.floatArray_getitem( tmpNormal, 2 );
//                normal.set( nx, ny, nz );
//                normal.scale( s );
//                fa.setSize(0.025);
//                fa.setFrom( pos.getX(), pos.getY(), pos.getZ() );
//                fa.setTo( pos.getX() + normal.getX(), pos.getY()  + normal.getY(), pos.getZ() + normal.getZ() );            
//                fa.draw(gl);
//                gl.glPushMatrix();
//                gl.glTranslated( pos.getX(), pos.getY(), pos.getZ() );
//                EasyViewer.glut.glutSolidSphere(0.015, 20, 10);
//                gl.glPopMatrix();
//            }
        }
    }
    
    private FancyArrow fa = new FancyArrow();
    
    private FlatMatrix4d M = new FlatMatrix4d();

    /**
     * Sets the provided homogeneous 4x4 matrix A using the current position and orientation of the body ODE rigid body b 
     * @param b
     * @param A
     */
    public static void setFlatMatrix( DBody b, FlatMatrix4d A ) {
        DVector3C p = b.getPosition();
        DMatrix3C R = b.getRotation();
        Matrix4d m = A.getBackingMatrix();
        m.setIdentity();
        for (int i = 0; i < 3; i++ ) {
            for ( int j = 0; j < 3; j ++ ) {
                m.setElement(i, j, R.get(i, j) );
            }
            m.setElement(i, 3, p.get(i) );
        }
    }
    


    /**
     * Initializes dynamic objects. These geometries have an body and simulation
     * steps apply various forces to them. Simulation may move these objects or
     * change their orientation as they e.g. collide to each other.
     */
    private void initObjects() {
        // Create a sphere geometry and set it to body (dynamic object)
        // Sphere radius is 1. Position sphere above the ground
        
    	/*
        sphereBody = OdeHelper.createBody( world );
        sphereMass = OdeHelper.createMass();
        sphereMass.setSphere( 1, 1 ); 
        // TODO: density 1000 or 1 ???  not clear!  perhaps that's why people call adjustMass (i.e., to scale)
        //sphereMass.adjust(1);
        sphereBody.setMass( sphereMass );
        sphere = OdeHelper.createSphere( space, 1 );
        sphere.setBody( sphereBody );
        */
   
        boxBody = OdeHelper.createBody( world );
        boxMass = OdeHelper.createMass();
        boxMass.setBox( 1, 0.3, 0.4, 0.6 ); 
        //boxMass.adjust(1); // gross... but not working nicely...
        // TODO: density 1000 or 1 ???  not clear!  perhaps that's why people call adjustMass (i.e., to scale)
        boxBody.setMass( boxMass );
        box = OdeHelper.createBox( space, 0.3, 0.4, 0.6 );
        box.setBody( boxBody );        

        
        setInitialObjectPositions();
    }
    
    /**
     * Sets the position and velocity of objects in the scene
     */
    private void setInitialObjectPositions() {
        DQuaternion q = new DQuaternion(1,0,0,0);
        
        /*
        sphereBody.setPosition( 1,2,-1 );
        sphereBody.setQuaternion( q );
        sphereBody.setLinearVel( 0,0,0 );
        sphereBody.setAngularVel( 0,0,0 );
        */

        box.setPosition( -1,2,-1 );
        box.setQuaternion( q );
        boxBody.setLinearVel( 0,0,0 );
        boxBody.setAngularVel( 0,0,0 );
        
        for ( BoxBodyODE b : bodies ) {
            b.reset();
        }
    }
    
    double elapsed = 0;
    
    
    /**
     * Steps the simulation ahead.
     */
    public void step() {
        int ss = subSteps.getValue();
        double h = stepSize.getValue() / ss;
        DVector3 g = new DVector3( 0, enableGravity.getValue() ? gravity.getValue() : 0, 0 );
        world.setGravity(g);
        world.setERP( worldERP.getValue() );
        world.setCFM( worldCFM.getValue() );
        for ( int i = 0; i < ss; i++ ) {  
            
            space.collide( 0, myNearCallback );

            applyJointTorques(root);

            //world.quickStep(h);
            world.step(h);
            elapsed += h;

            // extract all contact information for display!!!
            if ( i == ss-1 ) {
                // on our last step, grab all the necessary information to draw the contact 
                // before the contact group is emptied!
            }
            
            // remove all contact joints
            contactgroup.empty();
        }
        text = "time = " + elapsed;
    }   

    /**
     * Resets the simulation to its initial configuration
     */
    public void reset() {
        setInitialObjectPositions();
        elapsed = 0;
    }
    
    private DNearCallback myNearCallback = new DNearCallback() {
        @Override
        public void call(Object data, DGeom o1, DGeom o2) {
            nearCallback(data, o1, o2);
        }
    };

    /** 
     * this is called by dSpaceCollide when two objects in space are
     * potentially colliding.
     */
    private void nearCallback (Object data, DGeom o1, DGeom o2) {
        /* exit without doing anything if the two bodies are connected by a joint */
        DBody b1,b2;
        //dContact contact;
        final int maxContacts = 1;
        DContactBuffer contacts = new DContactBuffer(maxContacts);
        
        b1 = o1.getBody();
        b2 = o2.getBody();
        if ( b1 != null && b2 != null && !enableContacts.getValue() ) return;
        if ( b1 != null && b2 != null && OdeHelper.areConnected(b1,b2)) return;        
        if ( b1 != null && nonCollidable.contains( b1 ) ) return; 
        if ( b2 != null && nonCollidable.contains( b2 ) ) return; 
        
        int collisions = OdeHelper.collide( o1,o2,maxContacts,contacts.getGeomBuffer() );
        if ( collisions > 0 ) { //&contact.geom,sizeof(dContactGeom))) {
            for ( int i = 0; i < collisions; i++ ) {
                DContact contact = contacts.get(i);
                //contact.surface.mode = 0;
                contact.surface.mode = OdeConstants.dContactApprox1 | OdeConstants.dContactSoftCFM;
                contact.surface.mu = friction.getValue();
                //contact.surface.mu2 = 0;
                contact.surface.soft_cfm = 0.01;
                DContactJoint c = OdeHelper.createContactJoint ( world, contactgroup, contact);
                c.attach( b1, b2 );
                // c.setFeedback( fbPool ); // TODO: save these in a list... 
            }
        }
    }    
    
}