package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Defines a 3D pose in a fixed-frame as {@code YoFramePose3D} but use the Euler angles to represent the orientation part.
 * 
 * @see YoFramePose3D
 * @see YoFrameYawPitchRoll
 */
public class YoFramePoseUsingYawPitchRoll implements ReferenceFrameHolder, Clearable
{
   private final YoFramePoint3D position;
   private final YoFrameYawPitchRoll orientation;

   private final FrameQuaternion tempFrameOrientation = new FrameQuaternion();

   public YoFramePoseUsingYawPitchRoll(YoFramePoint3D position, YoFrameYawPitchRoll orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      this.position = position;
      this.orientation = orientation;
   }

   public YoFramePoseUsingYawPitchRoll(String prefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(prefix, "", frame, registry);
   }

   public YoFramePoseUsingYawPitchRoll(String prefix, String suffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      position = new YoFramePoint3D(prefix, suffix, frame, registry);
      orientation = new YoFrameYawPitchRoll(prefix, suffix, frame, registry);
   }

   public YoFramePoint3D getPosition()
   {
      return position;
   }

   public YoFrameYawPitchRoll getOrientation()
   {
      return orientation;
   }

   public void getFramePose(FixedFramePose3DBasics framePoseToPack)
   {
      orientation.getFrameOrientationIncludingFrame(tempFrameOrientation);

      framePoseToPack.setPosition(position);
      framePoseToPack.setOrientation(tempFrameOrientation);
   }

   public void getFramePoseIncludingFrame(FramePose3DBasics framePoseToPack)
   {
      framePoseToPack.setToZero(getReferenceFrame());
      getFramePose(framePoseToPack);
   }

   public void getPose(RigidBodyTransform rigidBodyTransformToPack)
   {
      orientation.getFrameOrientationIncludingFrame(tempFrameOrientation);
      rigidBodyTransformToPack.setRotation(tempFrameOrientation);
      rigidBodyTransformToPack.setTranslation(position);
   }

   public void set(FramePose3DReadOnly framePose)
   {
      framePose.checkReferenceFrameMatch(getReferenceFrame());

      position.set(framePose.getPosition());
      orientation.set(framePose.getOrientation());
   }

   public void setMatchingFrame(FramePose3DReadOnly framePose)
   {
      position.setMatchingFrame(framePose.getPosition());
      orientation.setMatchingFrame(framePose.getOrientation());
   }

   /**
    * Sets this frame pose to the origin of the passed in reference frame.
    * 
    * @param referenceFrame
    */
   public void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      position.setFromReferenceFrame(referenceFrame);
      orientation.setFromReferenceFrame(referenceFrame);
   }

   public void setPosition(FramePoint3DReadOnly framePoint)
   {
      position.set(framePoint);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      this.position.set(position);
   }

   public void setOrientation(FrameQuaternionReadOnly frameOrientation)
   {
      orientation.set(frameOrientation);
   }

   public void setOrientation(QuaternionReadOnly quaternion)
   {
      orientation.set(quaternion);
   }

   public void set(FramePoint3DReadOnly framePoint, FrameQuaternionReadOnly frameOrientation)
   {
      position.set(framePoint);
      orientation.set(frameOrientation);
   }

   public void set(YoFramePoseUsingYawPitchRoll yoFramePose)
   {
      set(yoFramePose.getPosition(), yoFramePose.getOrientation().getFrameOrientation());
   }

   public void setMatchingFrame(FramePoint3DReadOnly framePoint, FrameQuaternionReadOnly frameOrientation)
   {
      position.setMatchingFrame(framePoint);
      orientation.setMatchingFrame(frameOrientation);
   }

   public void setPosition(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   public void setXYZ(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   public void setXYZ(double[] pos)
   {
      setXYZ(pos[0], pos[1], pos[2]);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      orientation.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setYaw(double yaw)
   {
      orientation.setYaw(yaw);
   }

   public void setPitch(double pitch)
   {
      orientation.setPitch(pitch);
   }

   public void setRoll(double roll)
   {
      orientation.setRoll(roll);
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      orientation.setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public void setXYZYawPitchRoll(double[] pose)
   {
      setXYZ(pose[0], pose[1], pose[2]);
      setYawPitchRoll(pose[3], pose[4], pose[5]);
   }

   @Override
   public void setToNaN()
   {
      position.setToNaN();
      orientation.setToNaN();
   }

   @Override
   public void setToZero()
   {
      position.setToZero();
      orientation.setToZero();
   }

   @Override
   public boolean containsNaN()
   {
      return position.containsNaN() || orientation.containsNaN();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return position.getReferenceFrame();
   }

   public void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      position.attachVariableChangedListener(variableChangedListener);
      orientation.attachVariableChangedListener(variableChangedListener);
   }

   public double getDistance(YoFramePoseUsingYawPitchRoll goalYoPose)
   {
      return position.distance(goalYoPose.getPosition());
   }

   public void setX(double x)
   {
      position.setX(x);
   }

   public void setY(double y)
   {
      position.setY(y);
   }

   public void setZ(double z)
   {
      position.setZ(z);
   }

   public double getX()
   {
      return getPosition().getX();
   }

   public double getY()
   {
      return getPosition().getY();
   }

   public double getZ()
   {
      return getPosition().getZ();
   }

   public double getRoll()
   {
      return getOrientation().getRoll().getDoubleValue();
   }

   public double getPitch()
   {
      return getOrientation().getPitch().getDoubleValue();
   }

   public double getYaw()
   {
      return getOrientation().getYaw().getDoubleValue();
   }

   public void add(YoFramePoseUsingYawPitchRoll yoFramePose)
   {
      getPosition().add(yoFramePose.getPosition());
      getOrientation().add(yoFramePose.getOrientation());
   }

   public YoDouble getYoX()
   {
      return getPosition().getYoX();
   }

   public YoDouble getYoY()
   {
      return getPosition().getYoY();
   }

   public YoDouble getYoZ()
   {
      return getPosition().getYoZ();
   }

   public YoDouble getYoPitch()
   {
      return getOrientation().getPitch();
   }

   public YoDouble getYoRoll()
   {
      return getOrientation().getRoll();
   }

   public YoDouble getYoYaw()
   {
      return getOrientation().getYaw();
   }

   /**
    * Creates a copy of {@code this} by finding the duplicated {@code YoVariable}s in the given
    * {@link YoVariableRegistry}.
    * <p>
    * This method does not duplicate {@code YoVariable}s. Assuming the given registry is a duplicate
    * of the registry that was used to create {@code this}, this method searches for the duplicated
    * {@code YoVariable}s and use them to duplicate {@code this}.
    * </p>
    *
    * @param newRegistry YoVariableRegistry to duplicate {@code this} to.
    * @return the duplicate of {@code this}.
    */
   public YoFramePoseUsingYawPitchRoll duplicate(YoVariableRegistry newRegistry)
   {
      return new YoFramePoseUsingYawPitchRoll(position.duplicate(newRegistry), orientation.duplicate(newRegistry));
   }
}
