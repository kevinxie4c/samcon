package odemocap;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.io.File;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Random;

import com.jogamp.opengl.DebugGL2;
import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple4d;
import javax.vecmath.Vector3d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.BoxRoom;
import mintools.viewer.EasyViewer;
import mintools.viewer.FancyAxis;
import mintools.viewer.FlatMatrix4d;
import mintools.viewer.Interactor;
import mintools.viewer.SceneGraphNode;
import mintools.viewer.ShadowMap;

import org.ode4j.math.*;

/**
 * Assignment 3 base application
 * 
 * @author kry
 */
public class OdeMocapApp implements SceneGraphNode, Interactor {

	/**
	 * Entry point for application
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		new OdeMocapApp();
	}

	private EasyViewer ev;

	private SkeletonNode root;

	private BVHData bvhData = new BVHData();

	private C3DData c3dData = new C3DData();

	private ODESimulation odeSim = new ODESimulation();

	private boolean stepRequest = false;

	private boolean resetRequest = false;

	private boolean setCurrentPoseRequest = false;

	/**
	 * Creates a shadow map 1024x1024. This number can be reduced to improve
	 * performance but probably can't go too much higher.
	 */
	private ShadowMap shadowMap = new ShadowMap(2048);

	SkeletonNode vRoot;
	BVHData vData = new BVHData();
	String bvhFile = "";

	/**
	 * Creates the application / scene instance
	 */
	public OdeMocapApp() {

		// NOTE: testing with the first entries of the CMU database
		// is probably not ideal. One would imagine that the quality
		// of the data improves over time. Likewise, the 01_01 entry
		// is flagged in red, probably meaning that it has errors.

		// If using CMU database, use a scale of 0.06 (what crazy units are these?)
		// otherwise use the default (i.e., 0.01 to convert cm to meters)
		// C3D data appears to be consistently in mm thus the scale of 0.001

		// If using the natural point demo file, the translation axes
		// appear to be swapped. This may not be true for newer data.

		int which = 1;

		// String bvhFile = "";
		String c3dFile = "";
		if (which == 0) {
			bvhFile = "data\\OptiTrack-IITSEC2007.bvh";
			c3dFile = "data\\OptiTrack-IITSEC2007.javabin";
		} else if (which == 1) {
			bvhFile = "data/Cyrus_Take6.bvh";
			// c3dFile = "data\\01_01.javabin";
		} else if (which == 2) {
			bvhFile = "data/jon_take4.bvh";
			c3dFile = "data/jon_take4.javabin";
		} else if (which == 3) {
			String[] filename = { "2009-02-12_17-50-45_Take 1-ChrisWalking", "2009-02-12_17-58-13_Take 2-ChrisSilly(2)",
					"2009-02-12_18-03-04_Take 3-ChrisSilly2(1)", "2009-02-12_18-04-56_Take 4-ChrisStopping",
					"2009-02-12_18-29-37_Take 9", "2009-02-12_18-31-02_Take 10", "2009-02-12_18-33-56_Take 11",
					"2009-02-12_18-36-45_Take 12" };

			bvhFile = "data\\" + filename[5] + "-SJT.bvh";
			c3dFile = "data\\" + filename[5] + ".javabin";
		}

		// bvhFile = "data/02_02.bvh";
		//bvhFile = "data/Cyrus_Take7.bvh";
		bvhFile = "data/walk.bvh";
		bvhData.load(bvhFile);

		// c3dData.load( c3dFile );
		root = bvhData.root;
		frame.setMaximum(bvhData.getNumFrames() - 1);

		System.out.println("computing local frames");

		ev = new EasyViewer("Mocap + ODE Test", this, new Dimension(640, 480), new Dimension(640, 640));
		ev.addInteractor(this);
		ev.controlFrame.add("Shadow", shadowMap.getControls());

	}

	int sampleNum = 4000, saveNum = 1000, itNum = 40;
	//int sampleNum = 1, saveNum = 1, itNum = 20;
	int groupNum = 5, samRate = 1;
	int startFrame = 32;
	List<State> states;

	@Override
	public void init(GLAutoDrawable drawable) {
		drawable.setGL(new DebugGL2(drawable.getGL().getGL2()));
		GL2 gl = drawable.getGL().getGL2();
		gl.glEnable(GL.GL_BLEND);
		gl.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA);
		gl.glEnable(GL.GL_LINE_SMOOTH);
		gl.glEnable(GL2.GL_POINT_SMOOTH);
		if (root != null)
			root.init(drawable);
		shadowMap.init(drawable);

		double s = scalebvh.getValue();
		/*
		 * odeSim.createSkeleton( root, s ); odeSim.reset();
		 * 
		 * // DISABLE creation of joints for testing... odeSim.createSkeletonJoints(
		 * root );
		 */

		frame.setValue(startFrame);
		frame.setMinimum(startFrame);
		vData.load(bvhFile);
		vRoot = vData.root;
		if (vRoot != null)
			vRoot.init(drawable);
		odeSim.createSkeleton(vRoot, s);
		odeSim.reset();
		odeSim.createSkeletonJoints(vRoot);
		vData.setSkeletonPose(startFrame - 1);
		vRoot.computeTransforms();
		vRoot.copyEwfrombToPrevious();
		vData.setSkeletonPose(startFrame);
		vRoot.computeTransforms();
		odeSim.setCurrentPose(vRoot);
		odeSim.setCurrentPoseVelocity(vRoot, 0.01);

		List<State> refStateList = new ArrayList<State>();
		List<PhyProp> refPhyPropList= new ArrayList<PhyProp>();
		
		refStateList.add(null);
		refPhyPropList.add(null);
		for (int i = 1; i < itNum; ++i) {
			State tmp = new State();
			vData.setSkeletonPose(startFrame + i * groupNum);
			//vData.setSkeletonPose(0);
			vRoot.computeTransforms();
			odeSim.setCurrentPose(vRoot);
			odeSim.setCurrentPoseVelocity(vRoot, 0.01);
			tmp.save(vData.snodeList);
			refStateList.add(tmp);
			refPhyPropList.add(new PhyProp(vData.snodeList));
		}

		/*
		 * vData.load("data/Cyrus_Take6.bvh"); vRoot = vData.root; if ( vRoot != null )
		 * vRoot.init(drawable); odeSim.createSkeleton( vRoot, s ); odeSim.reset();
		 * odeSim.createSkeletonJoints( vRoot );
		 */
		vData.setSkeletonPose(startFrame - 1);
		vRoot.computeTransforms();
		vRoot.copyEwfrombToPrevious();
		vData.setSkeletonPose(startFrame);
		vRoot.computeTransforms();
		odeSim.setCurrentPose(vRoot);
		odeSim.setCurrentPoseVelocity(vRoot, 0.01);

		ArrayList<ArrayList<Sample>> sm = new ArrayList<ArrayList<Sample>>();
		ArrayList<Sample> tmplist = new ArrayList<Sample>();
		Sample tmpSample = new Sample();
		tmpSample.cost = 0;
		tmpSample.state.save(vData.snodeList);
		tmplist.add(tmpSample);
		sm.add(tmplist);
		for (int i = 1; i < itNum; ++i) {
			System.out.println("iteration " + i);
			ArrayList<Sample> slist = sm.get(i - 1);
			PriorityQueue<Sample> queue = new PriorityQueue<Sample>(comparator);
			for (int j = 0; j < slist.size(); ++j) {
				System.out.println("sample " + j);
				for (int k = 0; k < sampleNum / slist.size(); ++k) {
					slist.get(j).state.load(vData.snodeList);
					vData.setSkeletonPose(startFrame + i * groupNum);
					//vData.setSkeletonPose(0);
					Sample sample = new Sample();
					sample.prev = j;
					getSample(vData.snodeList, sample.dels);
					
					for (int t = 0; t < samRate * groupNum; ++t)
						odeSim.step();
					
					/*
					for (int t = 0; t < samRate; ++t)
						odeSim.step();
					*/
					sample.state.save(vData.snodeList);
					State target = refStateList.get(i);
					//sample.cost = pCost(sample.state, target);
					sample.cost = cost(vData.snodeList, refPhyPropList.get(i));
					// sample.cost = 0;
					queue.add(sample);
					if (queue.size() > saveNum)
						queue.remove();
				}
			}
			ArrayList<Sample> list = new ArrayList<Sample>();
			for (Sample it : queue)
				list.add(it);
			sm.add(list);
		}

		double min = Double.MAX_VALUE;
		List<Sample> minSm = null;
		for (Sample it : sm.get(sm.size() - 1)) {
			LinkedList<Sample> tmp = new LinkedList<Sample>();
			double cost = it.cost;
			tmp.addFirst(it);
			int index = it.prev;
			for (int i = sm.size() - 2; i >= 0; --i) {
				Sample t = sm.get(i).get(index);
				cost += t.cost;
				tmp.addFirst(t);
				index = t.prev;
			}
			if (cost < min) {
				min = cost;
				minSm = tmp;
			}
		}

		minSm.get(0).state.load(vData.snodeList);
		vData.setSkeletonPose(startFrame - 1);
		vRoot.computeTransforms();
		vRoot.copyEwfrombToPrevious();
		vData.setSkeletonPose(startFrame);
		vRoot.computeTransforms();
		odeSim.setCurrentPose(vRoot);
		odeSim.setCurrentPoseVelocity(vRoot, 0.01);
		states = new ArrayList<State>();
		State tState = new State();
		tState.save(vData.snodeList);
		states.add(tState);
		for (int i = 1; i < itNum; ++i) {
			setSample(vData.snodeList, minSm.get(i).dels);
			
			/*
			boolean first = true;
			for (int j = 0; j < groupNum; ++j) {
				tState = new State();
				if (first) {
					first = false;
					for (int k = 0; k < samRate; ++k)
						odeSim.step();
				}
				tState.save(vData.snodeList);
				states.add(tState);
			}
			*/
			
			for (int j = 0; j < groupNum; ++j) {
				tState = new State();
				for (int k = 0; k < samRate; ++k)
					odeSim.step();
				tState.save(vData.snodeList);
				states.add(tState);
			}
			
			
			
			System.out.println(minSm.get(i).cost);
		}

		/*
		 * samples = new ArrayList<Sample>(); for (int i = 1; i < 200; ++i) { Sample tmp
		 * = new Sample(); vData.setSkeletonPose(i); odeSim.step();
		 * tmp.state.save(vData.snodeList); samples.add(tmp);
		 * //tmp.state.load(vData.snodeList); }
		 */

		vData.setSkeletonPose(startFrame - 1);
		vRoot.computeTransforms();
		vRoot.copyEwfrombToPrevious();
		vData.setSkeletonPose(startFrame);
		vRoot.computeTransforms();
		odeSim.setCurrentPose(vRoot);
		odeSim.setCurrentPoseVelocity(vRoot, 0.01);

		accumulatorFrame.getBackingMatrix().setIdentity();

		boxRoom.setBoundingBox(new Vector3d(-4, odeSim.floorOffset, -4), new Vector3d(4, 8, 4));
		// boxRoom.setBoundingBox( new Vector3d( -4,-4,-4), new Vector3d(4,8,4) );

		// pairs.add( new Pair( 1847, 1946 ) );
		// pairs.add( new Pair( 2544, 4541 ) );

		// findPairs();
	}

	/**
	 * Reset the animation
	 */
	public void reset() {
		accumulatorFrame.getBackingMatrix().setIdentity();
		odeSim.reset();
	}

	/**
	 * Variable to keep track of the last frame number that was set so that the
	 * angles are not always set on every call. This lets us adjust the joint angle
	 * sliders to pose the character differently when the captured motion is not
	 * playing.
	 */
	private int lastFrameNumber = -1;

	private BoxRoom boxRoom = new BoxRoom();

	/**
	 * Conversion from centimeters to meters to bring bvh data into the units of the
	 * world. (note that BVH files of CMU database motions need a different
	 * conversion factor).
	 */
	private DoubleParameter scalebvh = new DoubleParameter("scale bvh", 0.01, 0.001, 1);

	/**
	 * Conversion from millimeters to meters to bring the c3d data into the units of
	 * the world.
	 */
	private DoubleParameter scalec3d = new DoubleParameter("scale c3d", 0.001, 0.001, 1);

	/**
	 * Size of the displayed markers in mm
	 */
	private DoubleParameter markerSize = new DoubleParameter("marker size", 10, 1, 100);

	/**
	 * A variable for starting and stopping the animation. The space bar in the
	 * display window will toggle this.
	 */
	private BooleanParameter play = new BooleanParameter("play animation", false);

	/**
	 * Sets ODE sim from mocap at every step if selected. Can cause unexpected
	 * results initialized with interpenetration!
	 */
	private BooleanParameter followMocap = new BooleanParameter("follow mocap", true);

	/**
	 * For enabling super cheap shadows...
	 */
	private BooleanParameter drawWithShadows = new BooleanParameter("draw with shadows", true);

	FancyAxis fa = new FancyAxis();

	private class Pair {
		int f1;
		int f2;

		public Pair(int f1, int f2) {
			this.f1 = f1;
			this.f2 = f2;
		}
	}

	private List<Pair> pairs = new ArrayList<Pair>();

	/**
	 * compute angles x,y,z such that M = Z*Y*X, where X,Y, and Z are the rotation
	 * matrices corresponding to the rotations of x,y, and z around the x,y, and z
	 * axes
	 * 
	 * @param M
	 * @param rpy
	 */
	public static void rpy(Matrix3d M, double[] rpy) {
		double x; // "phi" in Paul's book
		double y; // "theta"
		double z; // "psi"
		// he labels the columns of M as n,o,a
		Vector3d n = new Vector3d();
		Vector3d o = new Vector3d();
		Vector3d a = new Vector3d();
		M.getColumn(0, n);
		M.getColumn(1, o);
		M.getColumn(2, a);
		z = Math.atan2(n.y, n.x);
		y = Math.atan2(-n.z, Math.cos(z) * n.x + Math.sin(z) * n.y);
		x = Math.atan2(Math.sin(z) * a.x - Math.cos(z) * a.y, -Math.sin(z) * o.x + Math.cos(z) * o.y);
		rpy[0] = x;
		rpy[1] = y;
		rpy[2] = z;
	}

	/**
	 * Compute rotation angles around the axes in the columns of matrix axes such
	 * that these rotations give matrix M. Angles are stored in rpy.
	 * 
	 * @param axes
	 * @param M
	 * @param rpy
	 */
	public static void rpy(Matrix3d axes, Matrix3d M, double[] rpy) {
		Matrix3d zyxMat = new Matrix3d();
		zyxMat.mulTransposeLeft(axes, M);
		zyxMat.mul(zyxMat, axes);
		rpy(zyxMat, rpy);
		// if the axes passed in are left-handed, negate the rpy angles
		if (axes.determinant() < 0) {
			rpy[0] = -rpy[0];
			rpy[1] = -rpy[1];
			rpy[2] = -rpy[2];
		}
	}

	/**
	 * Combines a given transformation with the transformation in the root node of
	 * the skeleton, and sets the skeleton node parameters. That is, if the root
	 * node has a transform T, then the root node after calling this method will
	 * have parameters which will produce the transform A time T.
	 * 
	 * @param A
	 *            the accumulated transformation to combine.
	 * @param node
	 *            the root node of the skeleton
	 */
	public static void bakeMatrixIntoSkeletonNode(Matrix4d A, SkeletonNode node) {
		Matrix3d axes = new Matrix3d();
		axes.setColumn(2, new double[] { 0, 0, 1 });
		axes.setColumn(1, new double[] { 1, 0, 0 });
		axes.setColumn(0, new double[] { 0, 1, 0 });
		Matrix4d m = new Matrix4d();
		node.getTransform(m);
		Matrix4d Am = new Matrix4d();
		Am.mul(A, m);
		Matrix3d R = new Matrix3d();
		Am.getRotationScale(R);
		double[] zxy = new double[3];
		rpy(axes, R, zxy);
		node.zRot.setValue(zxy[2]);
		node.xRot.setValue(zxy[1]);
		node.yRot.setValue(zxy[0]);
		node.xTrans.setValue(Am.m03);
		node.yTrans.setValue(Am.m13);
		node.zTrans.setValue(Am.m23);
	}

	private int fnumber;

	@Override
	public void display(GLAutoDrawable drawable) {
		if (resetRequest) {
			resetRequest = false;
			reset();
		}
		if (refindpairs) {
			refindpairs = false;
			findPairs();
		}
		if (play.getValue() || stepRequest) {
			stepRequest = false;
			frame.setValue(frame.getValue() + playSpeed.getValue());
			if (frame.getValue() == frame.getMaximum()) {
				frame.setValue(startFrame);
			}

			doTransition();

			if (root != null)
				root.computeTransforms();

			// if (vRoot!=null) vRoot.computeTransforms();
			// odeSim.step();

			/*
			 * if (frame.getValue() < samples.size()) {
			 * samples.get(frame.getValue()).state.load(vData.snodeList); }
			 */

			if (frame.getValue() - startFrame < states.size()) {
				states.get(frame.getValue() - startFrame).load(vData.snodeList);
			}
		}
		fnumber = (int) frame.getValue();
		if (root != null) {
			if (displayT.getValue()) {
				root.resetToZeroPose();
			} else {
				if (fnumber != lastFrameNumber) {
					lastFrameNumber = fnumber;
					bvhData.setSkeletonPose(fnumber);
					bakeMatrixIntoSkeletonNode(accumulatorFrame.getBackingMatrix(), root);

					vData.setSkeletonPose(fnumber);
				}
				if (recordbvh.getValue()) {
					bvhData.addCurrentPoseToTrajectory();
					savedTrajectorySize.setText("count = " + bvhData.getNumSavedFrames());
				}
			}
		}
		if (setCurrentPoseRequest || followMocap.getValue()) {
			setCurrentPoseRequest = false;
			bvhData.setSkeletonPose((int) Math.max(startFrame, frame.getValue() - 1));
			root.computeTransforms();
			root.copyEwfrombToPrevious();
			bvhData.setSkeletonPose((int) frame.getValue());
			root.computeTransforms();
			// odeSim.setCurrentPose(root);
			// odeSim.setCurrentPoseVelocity(root, 0.01);

			// vData.setSkeletonPose( (int)Math.max(0,frame.getValue()-1) );
			// vRoot.computeTransforms();
			// vRoot.copyEwfrombToPrevious();
			// vData.setSkeletonPose( (int) frame.getValue() );
			// vRoot.computeTransforms();
			// odeSim.setCurrentPose(root);
			// odeSim.setCurrentPoseVelocity(root, 0.01);
			if (frame.getValue() - startFrame < states.size()) {
				states.get(frame.getValue() - startFrame).load(vData.snodeList);
			}
		}

		GL2 gl = drawable.getGL().getGL2();

		if (!drawWithShadows.getValue()) {
			drawAllObjects(drawable, false);
		} else {
			// disable lights enabled by the easy viewer... the shadow map will enable
			// different lights...
			gl.glDisable(GL2.GL_LIGHT0);
			gl.glDisable(GL2.GL_LIGHT1);

			shadowMap.beginLightPass(drawable);
			drawAllObjects(drawable, true);
			shadowMap.endLightPass(drawable);

			// Draw ambient only...
			shadowMap.beginAmbientPass(drawable);
			drawAllObjects(drawable, false);
			shadowMap.endAmbientPass(drawable);

			// Draw diffuse with shadows
			shadowMap.beginShadowMapping(drawable);
			drawAllObjects(drawable, false);
			shadowMap.endShadowMapping(drawable);

			// Draw debugging information (Frustum, shadow map)
			shadowMap.displayDebug(drawable);
		}

		EasyViewer.beginOverlay(drawable);
		EasyViewer.printTextLines(drawable, odeSim.text);

		// draw the transitions?
		for (Pair p : pairs) {
			drawArc(drawable, p.f1, p.f2);
		}

		drawSimilarity(drawable, fnumber);

		int w = drawable.getSurfaceWidth();
		int h = drawable.getSurfaceHeight();

		gl.glColor4f(1, 1, 1, 1);
		gl.glBegin(GL.GL_LINES);
		gl.glVertex2d(10, h - 10);
		gl.glVertex2d(w - 10, h - 10);
		gl.glEnd();
		gl.glColor4f(.8f, 0, 0, .7f);
		gl.glPointSize(10f);
		gl.glBegin(GL.GL_POINTS);
		gl.glVertex2d(10 + (w - 20.0) * fnumber / frame.getMaximum(), h - 10);
		gl.glEnd();

		EasyViewer.endOverlay(drawable);

		if (play.getValue() || stepped) {
			stepped = false;
			if (record.getValue()) {
				// write the frame
				File file = new File("stills/" + dumpName + format.format(nextFrameNum) + ".png");
				nextFrameNum++;
				file = new File(file.getAbsolutePath().trim());
				ev.snapshot(drawable, file);
			}
		}
	}

	private void drawAllObjects(GLAutoDrawable drawable, boolean lightPass) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glPushMatrix();
		boxRoom.display(drawable);
		gl.glPopMatrix();

		odeSim.display(drawable);

		// draw the world reference frame
		// fa.setSize(0.25);
		// fa.draw(gl);

		double sc = scalec3d.getValue();
		double s = scalebvh.getValue();

		gl.glPushMatrix();

		gl.glScaled(s, s, s);
		// gl.glMultMatrixd( accumulatorFrame.asArray(), 0 );
		fa.setSize(0.25 / s);
		fa.draw(gl);

		if (drawc3d.getValue()) {
			gl.glPushMatrix();
			gl.glScaled(sc / s, sc / s, sc / s);
			displayC3D(drawable, fnumber);
			gl.glPopMatrix();
		}

		if (root != null && !lightPass) {
			root.display(drawable);

			// build the local frame and display it

			// Matrix4d m = tmpFlatMatrix.getBackingMatrix();
			// root.getTransform( m );
			// getFloorFrame( m );
			// gl.glPushMatrix();
			// gl.glMultMatrixd( tmpFlatMatrix.asArray(), 0 );
			// fa.setSize( 0.25 / s );
			// fa.draw(gl);
			//
			//
			// if ( drawc3dlocal.getValue() && fnumber < localc3d.length ) {
			// //lets draw the local frame positions and vectors
			// gl.glColor4d(1,0,0,0.7);
			// Point3d p = new Point3d();
			// Vector3d v = new Vector3d();
			// gl.glBegin( GL.GL_LINES );
			// for ( int i = 0; i < localc3d[fnumber].length; i+=6 ) {
			// p.set(localc3d[fnumber][i+0],localc3d[fnumber][i+1],localc3d[fnumber][i+2]);
			// v.set(localc3d[fnumber][i+3],localc3d[fnumber][i+4],localc3d[fnumber][i+5]);
			// v.scale(3);
			// gl.glVertex3d( p.x, p.y, p.z );
			// p.add(v);
			// gl.glVertex3d( p.x, p.y, p.z );
			// }
			// gl.glEnd();
			// }
			//
			// gl.glPopMatrix();
		}
		gl.glPopMatrix();
	}

	/**
	 * boolean to signal that the system was stepped and that a frame should be
	 * recorded if recording is enabled
	 */
	private boolean stepped = false;

	private BooleanParameter record = new BooleanParameter("record (press ENTER in canvas to toggle)", false);

	private String dumpName = "dump";

	private int nextFrameNum = 0;

	private NumberFormat format = new DecimalFormat("00000");

	private Point3d tmpP = new Point3d();

	private void displayC3D(GLAutoDrawable drawable, int fnumber) {
		GL2 gl = drawable.getGL().getGL2();
		if (c3dData.getNumFrames() > fnumber) {
			if (drawc3dpoints.getValue()) {
				gl.glDisable(GL2.GL_LIGHTING);
				gl.glColor4f(0, 0.7f, 0, 0.7f);
				gl.glPointSize(10);
				gl.glBegin(GL.GL_POINTS);
				for (int i = 0; i < c3dData.getNumMarkers(); i++) {
					c3dData.getMarker(fnumber, i, tmpP);
					gl.glVertex3d(tmpP.x, tmpP.y, tmpP.z);
				}
				gl.glEnd();
				gl.glEnable(GL2.GL_LIGHTING);
			} else {
				final float[] markerColor = new float[] { .8f, .8f, .8f, 1 };
				gl.glMaterialfv(GL.GL_FRONT_AND_BACK, GL2.GL_DIFFUSE, markerColor, 0);
				for (int i = 0; i < c3dData.getNumMarkers(); i++) {
					c3dData.getMarker(fnumber, i, tmpP);
					gl.glPushMatrix();
					gl.glTranslated(tmpP.x, tmpP.y, tmpP.z);
					EasyViewer.glut.glutSolidSphere(markerSize.getValue(), 16, 8);
					gl.glPopMatrix();
				}
			}
		}
	}

	/** Random number generator for deciding which transitions to take */
	private Random rand = new Random();

	private void doTransition() {
		if (doTransitions.getValue()) {
			int f = (int) frame.getValue();
			for (Pair p : pairs) {
				if (f >= p.f1 && f <= p.f1 + playSpeed.getValue() && !alwaysJumpBack.getValue()
						&& (rand.nextBoolean() || alwaysJumpForward.getValue()) && lastPairFrame != p.f2) {
					// forward transition
					// don't jump past last trip back!
					bvhData.setSkeletonPose(p.f1);
					Matrix4d m = tmpFlatMatrix.getBackingMatrix();
					root.getTransform(m);
					getFloorFrame(m);

					bvhData.setSkeletonPose(p.f2);
					Matrix4d m2 = tmpFlatMatrix2.getBackingMatrix();
					root.getTransform(m2);
					getFloorFrame(m2);

					m2.invert();
					m.mul(m2);
					accumulatorFrame.getBackingMatrix().mul(m);

					frame.setValue(p.f2 + (f - p.f1));
					break;
				} else if (f >= p.f2 && f <= p.f2 + playSpeed.getValue() && !alwaysJumpForward.getValue()
						&& ((rand.nextBoolean() || alwaysJumpBack.getValue()) || lastPairFrame == p.f2)) {
					// backward transition...
					// always take last jump back

					bvhData.setSkeletonPose(p.f2);
					Matrix4d m = tmpFlatMatrix.getBackingMatrix();
					root.getTransform(m);
					getFloorFrame(m);

					bvhData.setSkeletonPose(p.f1);
					Matrix4d m2 = tmpFlatMatrix2.getBackingMatrix();
					root.getTransform(m2);
					getFloorFrame(m2);

					m2.invert();
					m.mul(m2);
					accumulatorFrame.getBackingMatrix().mul(m);

					frame.setValue(p.f1 + (f - p.f2));
					break;
				}
			}
		}
	}

	private double[][] localc3d;
	private double[][] similarity;

	private void computeLocalFrames() {
		// first lets build the local frame c3d data and speed vectors
		int nFrames = c3dData.getNumFrames();

		localc3d = new double[nFrames][c3dData.getNumMarkers() * 6];

		double sc = scalec3d.getValue();
		double s = scalebvh.getValue();

		Point3d p1 = new Point3d();
		Point3d p2 = new Point3d();
		Point3d p3 = new Point3d();
		Vector3d v = new Vector3d();
		Vector3d vtmp = new Vector3d();

		// average 5 local velocities together to get a smoother picture
		int offset = 5;
		for (int i = 0; i < nFrames - offset; i++) {
			bvhData.setSkeletonPose(i);
			Matrix4d m = tmpFlatMatrix.getBackingMatrix();
			root.getTransform(m);
			getFloorFrame(m);
			m.invert();

			for (int j = 0; j < c3dData.getNumMarkers(); j++) {
				c3dData.getMarker(i, j, p1);
				p3.set(p1);
				v.set(0, 0, 0);
				for (int k = 0; k < offset; k++) {
					c3dData.getMarker(i + offset, j, p2);
					vtmp.sub(p2, p3);
					p3.set(p2);
					v.add(vtmp);
				}
				p1.scale(sc / s);
				v.scale(sc / s);
				m.transform(p1);
				m.transform(v);
				localc3d[i][j * 6 + 0] = p1.x;
				localc3d[i][j * 6 + 1] = p1.y;
				localc3d[i][j * 6 + 2] = p1.z;
				localc3d[i][j * 6 + 3] = v.x;
				localc3d[i][j * 6 + 4] = v.y;
				localc3d[i][j * 6 + 5] = v.z;
			}
		}
	}

	private void computeSimilarity() {
		int nFrames = c3dData.getNumFrames();

		System.out.println("computing similarity");
		similarity = new double[nFrames / simfac][nFrames / simfac];
		double vScaleForMetric = scaleForMetric.getValue();
		for (int i = 0; i < nFrames / simfac; i++) {
			for (int j = i; j < nFrames / simfac; j++) {
				for (int k = 0; k < localc3d[0].length; k++) {
					double val = localc3d[i * simfac][k] - localc3d[j * simfac][k];
					double term = val * val;
					if (k % 6 == 3 || k % 6 == 4 || k % 6 == 5) {
						term *= vScaleForMetric;
					}
					similarity[i][j] += term;
				}
				similarity[i][j] = Math.sqrt(similarity[i][j]);
				similarity[j][i] = similarity[i][j];
			}
		}

	}

	/** keep track of the last frame in a pair so we can force a back jump */
	int lastPairFrame = -1;

	private void findPairs() {
		System.out.println("finding pairs");
		pairs.clear();

		lastPairFrame = -1;

		double thresh = similarityThreshold.getValue();
		int mJump = minJump.getValue();
		double best = thresh;
		int bestIndex = -1;

		for (int i = 0; i < similarity[0].length; i++) {
			best = thresh;
			bestIndex = -1;
			for (int j = i + mJump; j < similarity[0].length; j++) {
				if (similarity[i][j] < thresh && similarity[i][j] < best) {
					bestIndex = j;
					best = similarity[i][j];
				}
			}
			if (bestIndex != -1) {
				pairs.add(new Pair(i * simfac, bestIndex * simfac));
				i += mJump;
				if (bestIndex * simfac > lastPairFrame) {
					lastPairFrame = bestIndex * simfac;
				}
			}
		}
		System.out.println("done");
	}

	/** reduce the number of frames that we actually compare to one in 5 */
	private int simfac = 5;

	private void drawSimilarity(GLAutoDrawable drawable, int f) {
		if (similarity == null)
			return;
		if (f / simfac >= similarity.length)
			return;
		int w = drawable.getSurfaceWidth();
		int h = drawable.getSurfaceHeight();
		double max = similarity[f / simfac].length;
		GL2 gl = drawable.getGL().getGL2();
		double ss = similarityScale.getValue();

		gl.glColor4f(1, 0, 0, 0.7f);
		gl.glBegin(GL.GL_LINE_STRIP);
		for (int i = 0; i < max; i++) {
			gl.glVertex2d(10 + (w - 20) * (i) / max, h - 10 - ss * similarity[f / simfac][i]);
		}
		gl.glEnd();

		gl.glColor4f(1, 1, 1, 1);
		gl.glBegin(GL.GL_LINES);
		gl.glVertex2d(10, h - 10 - ss * similarityThreshold.getValue());
		gl.glVertex2d(w - 10, h - 10 - ss * similarityThreshold.getValue());
		gl.glEnd();
	}

	private void drawArc(GLAutoDrawable drawable, int s, int e) {
		double r = (e - s) / 2.0;
		double m = (e + s) / 2.0;
		int w = drawable.getSurfaceWidth();
		int h = drawable.getSurfaceHeight();
		double max = frame.getMaximum();
		GL2 gl = drawable.getGL().getGL2();
		gl.glColor4d(0, 0.7, 0, 0.7);
		gl.glBegin(GL.GL_LINE_STRIP);
		for (int i = 0; i <= 20; i++) {
			double angle = i * Math.PI / 20;
			gl.glVertex2d(10 + (m + r * Math.cos(angle)) * (w - 20) / max,
					h - 10 - (Math.sin(angle)) * (20 + 100 * r / max));
		}
		gl.glEnd();
	}

	private FlatMatrix4d tmpFlatMatrix = new FlatMatrix4d();
	private FlatMatrix4d tmpFlatMatrix2 = new FlatMatrix4d();

	private FlatMatrix4d accumulatorFrame = new FlatMatrix4d();

	/**
	 * Get the body frame pointing in the direction of the root that sits on the
	 * floor.
	 * 
	 * @param m
	 */
	public void getFloorFrame(Matrix4d m) {
		Matrix3d r = new Matrix3d();
		m.getRotationScale(r);
		m.m13 = 0;
		Vector3d tmp1 = new Vector3d();
		Vector3d tmp2 = new Vector3d(0, 1, 0);
		Vector3d tmp3 = new Vector3d(0, 0, 1);
		tmp3.set(0, 0, 1);
		r.transform(tmp3);
		tmp3.y = 0;
		tmp3.normalize();
		tmp1.cross(tmp2, tmp3);
		r.setColumn(0, tmp1);
		r.setColumn(1, tmp2);
		r.setColumn(2, tmp3);
		m.setRotation(r);
	}

	BooleanParameter displayT = new BooleanParameter("show T", false);
	IntParameter frame = new IntParameter("frame number", 0, 0, 1);

	IntParameter playSpeed = new IntParameter("play speed", 1, 0, 10);

	BooleanParameter drawc3d = new BooleanParameter("drawc3d", false);
	BooleanParameter drawc3dpoints = new BooleanParameter("drawc3d points (spheres)", false);
	BooleanParameter drawc3dlocal = new BooleanParameter("drawc3d local", false);

	BooleanParameter doTransitions = new BooleanParameter("do transitions", true);

	DoubleParameter similarityScale = new DoubleParameter("scale similarity", 0.001, 0.001, 1);
	DoubleParameter similarityThreshold = new DoubleParameter("similarity threshold", 50, 1, 1000);
	DoubleParameter scaleForMetric = new DoubleParameter("v scale for metric", 1, 0.01, 100);

	IntParameter minJump = new IntParameter("min jump", 30, 30, 300);

	BooleanParameter alwaysJumpBack = new BooleanParameter("always jump back", false);
	BooleanParameter alwaysJumpForward = new BooleanParameter("always jump forward", false);

	/**
	 * Checkbox to enable recording of a new bvh trajectory
	 */
	BooleanParameter recordbvh = new BooleanParameter("record bvh", false);

	/**
	 * This label is used to show the number of frames saved to the new bvh
	 * trajectory
	 */
	JLabel savedTrajectorySize = new JLabel("count = 0");

	@Override
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.add(record.getControls());

		vfp.add(recordbvh.getControls());
		vfp.add(savedTrajectorySize);
		JButton clear = new JButton("clear recorded bvh trajectory");
		clear.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				bvhData.clearSavedTrajectory();
				savedTrajectorySize.setText("count = 0");
			}
		});
		vfp.add(clear);

		JButton save = new JButton("save bvh to data/out.bvh");
		save.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				bvhData.save("data/out.bvh");
			}
		});
		vfp.add(save);

		vfp.add(frame.getSliderControls());
		vfp.add(scalebvh.getSliderControls(true));
		vfp.add(scalec3d.getSliderControls(true));
		vfp.add(markerSize.getSliderControls(true));
		vfp.add(SkeletonNode.getStaticControls());
		vfp.add(displayT.getControls());

		vfp.add(play.getControls());
		vfp.add(followMocap.getControls());
		vfp.add(playSpeed.getSliderControls());

		vfp.add(drawc3d.getControls());
		vfp.add(drawc3dpoints.getControls());
		vfp.add(drawc3dlocal.getControls());

		vfp.add(similarityScale.getSliderControls(true));
		vfp.add(similarityThreshold.getSliderControls(true));
		vfp.add(minJump.getControls());
		vfp.add(scaleForMetric.getSliderControls(true));

		vfp.add(alwaysJumpBack.getControls());
		vfp.add(alwaysJumpForward.getControls());

		vfp.add(doTransitions.getControls());

		vfp.add(drawWithShadows.getControls());

		if (root != null) {
			CollapsiblePanel cp = new CollapsiblePanel(root.getControls());
			cp.collapse();
			vfp.add(cp);
		}
		vfp.add(odeSim.getControls());

		return vfp.getPanel();
	}

	boolean refindpairs = false;

	@Override
	public void attach(Component component) {
		component.addKeyListener(new KeyAdapter() {
			@Override
			public void keyPressed(KeyEvent e) {
				if (e.getKeyCode() == KeyEvent.VK_SPACE) {
					play.setValue(!play.getValue());
				} else if (e.getKeyCode() == KeyEvent.VK_P) {
					refindpairs = true;
				} else if (e.getKeyCode() == KeyEvent.VK_D) {
					computeSimilarity();
					System.out.println("done");
				} else if (e.getKeyCode() == KeyEvent.VK_R) {
					resetRequest = true;
					setCurrentPoseRequest = true;
				} else if (e.getKeyCode() == KeyEvent.VK_S) {
					stepRequest = true;
				} else if (e.getKeyCode() == KeyEvent.VK_E) {
					setCurrentPoseRequest = true;
				} else if (e.getKeyCode() == KeyEvent.VK_F) {
					followMocap.setValue(!followMocap.getValue());
				} else if (e.getKeyCode() == KeyEvent.VK_LEFT) {
					frame.setValue(frame.getValue() - 1);
				} else if (e.getKeyCode() == KeyEvent.VK_RIGHT) {
					frame.setValue(frame.getValue() + 1);
				} else if (e.getKeyCode() == KeyEvent.VK_NUMPAD4) {
					frame.setValue(frame.getValue() - 10);
				} else if (e.getKeyCode() == KeyEvent.VK_NUMPAD6) {
					frame.setValue(frame.getValue() + 10);
				}
				if (e.getKeyCode() == KeyEvent.VK_ESCAPE) {
					ev.stop(); // quit
				}
				if (e.getKeyCode() != KeyEvent.VK_ESCAPE)
					ev.redisplay();
			}
		});
	}

	double norm(Tuple2d q) {
		return Math.sqrt(q.x * q.x + q.y * q.y);
	}
	
	double norm(Tuple3d q) {
		return Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
	}

	double norm(Tuple4d q) {
		return Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	}

	double quatDist(DQuaternionC q1, DQuaternionC q2) {
		Quat4d qa = new Quat4d(q1.get0(), q1.get1(), q1.get2(), q1.get3());
		Quat4d qb = new Quat4d(q2.get0(), q2.get1(), q2.get2(), q2.get3());
		qa.mulInverse(qb);
		double s = Math.log(norm(qa));
		Vector3d v = new Vector3d(qa.x, qa.y, qa.z);
		double a = qa.w;
		v.normalize();
		v.scale(Math.acos(a / norm(qa)));
		return Math.sqrt(s * s + norm(v) * norm(v));
	}

	double pCost(State s1, State s2) {
		double a = 0, b = 0;
		for (int i = 0; i < s1.quat.size(); ++i) {
			if (s1.quat.get(i) == null || s2.quat.get(i) == null)
				continue;
			a += quatDist(s1.quat.get(i), s2.quat.get(i));
		}
		for (int i = 0; i < s1.aVel.size(); ++i) {
			DVector3C v1 = s1.aVel.get(i);
			DVector3C v2 = s2.aVel.get(i);
			if (v1 == null || v2 == null)
				continue;
			b += norm(new Vector3d(v1.get0() - v2.get0(), v1.get1() - v2.get1(), v1.get2() - v2.get2()));
		}
		return a + 0.1 * b;
	}

	double cost(List<SkeletonNode> sl, PhyProp phyProp) {
		double res = 0;
		int i = 0;
		for (SkeletonNode a: sl) {
			if (a.odeBody == null || a.odeBody.body == null)
				continue;
			if (a.name.endsWith("Ankle") || a.name.endsWith("Head") || a.name.endsWith("Wrist")) {
				DVector3C v1 = a.odeBody.body.getPosition();
				Point3d v2 = phyProp.posList.get(i);
				res += norm(new Vector3d(v1.get0() - v2.x, v1.get1() - v2.y, v1.get2() - v2.z));
			}
			++i;
		}
		return res;
	}

	double samParaScale = 5;

	void getSample(List<SkeletonNode> snlist, List<Double> dels) {
		dels.clear();
		for (SkeletonNode sn : snlist) {
			if (sn.chanlist == null)
				continue;
			for (String chanName : sn.chanlist) {
				DoubleParameter ref = null;
				if (chanName.equals("Xposition")) {
					ref = sn.xTrans;
				} else if (chanName.equals("Yposition")) {
					ref = sn.yTrans;
				} else if (chanName.equals("Zposition")) {
					ref = sn.zTrans;
				} else if (chanName.equals("Xrotation")) {
					ref = sn.xRot;
				} else if (chanName.equals("Yrotation")) {
					ref = sn.yRot;
				} else if (chanName.equals("Zrotation")) {
					ref = sn.zRot;
				}

				double tmp = ref.getValue();
				String name = sn.name;
				double scale = samParaScale;
				if (chanName.equals("Xrotation"))
					scale /= 2;
				if (name.endsWith("Ankle") || name.endsWith("Hip") || name.endsWith("Knee"))
					do {
						tmp = ref.getValue() + (Math.random() - 0.5) / scale;
					} while (tmp > ref.getMaximum() || tmp < ref.getMinimum());
				//tmp = ref.getValue();
				ref.setValue(tmp);
				dels.add(tmp);
			}
		}
	}

	void setSample(List<SkeletonNode> snlist, List<Double> dels) {
		int i = 0;
		for (SkeletonNode sn : snlist) {
			if (sn.chanlist == null)
				continue;
			for (String chanName : sn.chanlist) {
				DoubleParameter ref = null;
				if (chanName.equals("Xposition")) {
					ref = sn.xTrans;
				} else if (chanName.equals("Yposition")) {
					ref = sn.yTrans;
				} else if (chanName.equals("Zposition")) {
					ref = sn.zTrans;
				} else if (chanName.equals("Xrotation")) {
					ref = sn.xRot;
				} else if (chanName.equals("Yrotation")) {
					ref = sn.yRot;
				} else if (chanName.equals("Zrotation")) {
					ref = sn.zRot;
				}
				ref.setValue(dels.get(i++));
			}
		}
	}

	static Comparator<Sample> comparator = new Comparator<Sample>() {

		@Override
		public int compare(Sample arg0, Sample arg1) {
			// TODO Auto-generated method stub
			return (int) (arg1.cost - arg0.cost);
		}

	};

}

class State {
	List<DQuaternionC> quat = new ArrayList<DQuaternionC>();
	List<DVector3C> aVel = new ArrayList<DVector3C>();
	List<DVector3C> pos = new ArrayList<DVector3C>();
	List<DVector3C> vel = new ArrayList<DVector3C>();

	void save(List<SkeletonNode> snlist) {
		quat.clear();
		aVel.clear();
		for (SkeletonNode it : snlist) {
			DQuaternion tmp = new DQuaternion();
			if (it.odeBody == null || it.odeBody.body == null) {
				quat.add(null);
				aVel.add(null);
				pos.add(null);
				vel.add(null);
			} else {
				tmp.set(it.odeBody.body.getQuaternion());
				quat.add(tmp);
				aVel.add(new DVector3(it.odeBody.body.getAngularVel()));
				pos.add(new DVector3(it.odeBody.body.getPosition()));
				vel.add(new DVector3(it.odeBody.body.getLinearVel()));
			}
		}
	}

	void load(List<SkeletonNode> snlist) {
		for (int i = 0; i < snlist.size(); ++i) {
			if (snlist.get(i).odeBody == null || snlist.get(i).odeBody.body == null)
				continue;
			snlist.get(i).odeBody.body.setQuaternion(quat.get(i));
			snlist.get(i).odeBody.body.setAngularVel(aVel.get(i));
			snlist.get(i).odeBody.body.setPosition(pos.get(i));
			snlist.get(i).odeBody.body.setLinearVel(vel.get(i));
		}
	}
}

class Sample {
	State state = new State();
	int prev;
	List<Double> dels = new ArrayList<Double>();
	double cost;
}

class PhyProp {
	List<Point3d> posList = new ArrayList<Point3d>();
	List<Double> massList = new ArrayList<Double>();
	
	PhyProp(List<SkeletonNode> snlist) {
		for (SkeletonNode sn: snlist) {
			if (sn.odeBody == null || sn.odeBody.body == null)
				continue;
			DVector3C p = sn.odeBody.body.getPosition();
			posList.add(new Point3d(p.get0(), p.get1(), p.get2()));
			massList.add(sn.odeBody.body.getMass().getMass());
		}
	}
}
