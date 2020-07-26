package us.ihmc.yoVariables.euclid.referenceFrame;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameYawPitchRollBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Defines a 3D representation as {@code YoFrameQuaternion} but using the Euler angles.
 */
public class YoFrameYawPitchRoll implements FixedFrameYawPitchRollBasics
{
   private final YoDouble yaw, pitch, roll; // This is where the data is stored. All operations must act on these numbers.
   private final ReferenceFrame referenceFrame;

   private final FrameQuaternion frameQuaternion = new FrameQuaternion();

   private boolean enableNotifications = true;

   /**
    * Creates a new {@code YoFrameYawPitchRoll}, initializes its angles to zero and its reference frame
    * to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param referenceFrame the reference frame for this {@code YoFrameYawPitchRoll}.
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameYawPitchRoll(String namePrefix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFrameYawPitchRoll} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this {@code YoFrameYawPitchRoll}.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameYawPitchRoll(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      yaw = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "yaw", nameSuffix), registry);
      pitch = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "pitch", nameSuffix), registry);
      roll = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "roll", nameSuffix), registry);

      this.referenceFrame = referenceFrame;
   }

   /**
    * Creates a new {@code YoFrameYawPitchRoll} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param yaw            an existing variable representing the yaw value of this
    *                       {@code YoFrameYawPitchRoll}.
    * @param pitch          an existing variable representing the pitch value of this
    *                       {@code YoFrameYawPitchRoll}.
    * @param roll           an existing variable representing the roll value of this
    *                       {@code YoFrameYawPitchRoll}.
    * @param referenceFrame the reference frame for this {@code YoFrameYawPitchRoll}.
    */
   public YoFrameYawPitchRoll(YoDouble yaw, YoDouble pitch, YoDouble roll, ReferenceFrame referenceFrame)
   {
      this.yaw = yaw;
      this.pitch = pitch;
      this.roll = roll;

      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      setYawPitchRoll(yaw, pitch, roll, true);
   }

   /**
    * Redirection to {@link #setYawPitchRoll(double, double, double)} while giving control over whether
    * to notify the listeners registered to this object.
    * 
    * @param yaw             the angle to rotate about the z-axis.
    * @param pitch           the angle to rotate about the y-axis.
    * @param roll            the angle to rotate about the x-axis.
    * @param notifyListeners whether to notify the listeners or not.
    */
   public void setYawPitchRoll(double yaw, double pitch, double roll, boolean notifyListeners)
   {
      this.yaw.set(yaw, notifyListeners);
      this.pitch.set(pitch, notifyListeners);
      this.roll.set(roll, notifyListeners);
   }

   /** {@inheritDoc} */
   @Override
   public void setYaw(double yaw)
   {
      this.yaw.set(yaw, enableNotifications);
   }

   /** {@inheritDoc} */
   @Override
   public void setPitch(double pitch)
   {
      this.pitch.set(pitch, enableNotifications);
   }

   /** {@inheritDoc} */
   @Override
   public void setRoll(double roll)
   {
      this.roll.set(roll, enableNotifications);
   }

   /**
    * Redirection to {@link #set(FrameOrientation3DReadOnly)} while giving control over whether to
    * notify the listeners registered to this object.
    * 
    * @param orientation     the new orientation. Not modified.
    * @param notifyListeners whether to notify the listeners or not.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   public void set(FrameOrientation3DReadOnly orientation, boolean notifyListeners)
   {
      enableNotifications = notifyListeners;
      set(orientation);
      enableNotifications = true;
   }

   /**
    * Redirection to {@link #set(Orientation3DReadOnly)} while giving control over whether to notify
    * the listeners registered to this object.
    * 
    * @param orientation     the new orientation. Not modified.
    * @param notifyListeners whether to notify the listeners or not.
    */
   public void set(Orientation3DReadOnly orientation, boolean notifyListeners)
   {
      enableNotifications = notifyListeners;
      set(orientation);
      enableNotifications = true;
   }

   /**
    * Redirection to {@link #setMatchingFrame(FrameOrientation3DReadOnly)} while giving control over
    * whether to notify the listeners registered to this object.
    * 
    * @param orientation     the new orientation. Not modified.
    * @param notifyListeners whether to notify the listeners or not.
    */
   public void setMatchingFrame(FrameOrientation3DReadOnly orientation, boolean notifyListeners)
   {
      enableNotifications = notifyListeners;
      setMatchingFrame(orientation);
      enableNotifications = true;
   }

   /**
    * Redirection to {@link #setMatchingFrame(FrameOrientation3DReadOnly)} while giving control over
    * whether to notify the listeners registered to this object.
    * 
    * @param referenceFrame  the reference frame of interest.
    * @param notifyListeners whether to notify the listeners or not.
    */
   public void setFromReferenceFrame(ReferenceFrame referenceFrame, boolean notifyListeners)
   {
      enableNotifications = notifyListeners;
      setFromReferenceFrame(referenceFrame);
      enableNotifications = true;
   }

   /**
    * Adds the angles of {@code other} to {@code this}.
    * 
    * @param other the other yaw-pitch-roll to add to {@code this}. Not modified.
    */
   public void add(YoFrameYawPitchRoll other)
   {
      yaw.add(other.getYaw());
      pitch.add(other.getPitch());
      roll.add(other.getRoll());
   }

   /** {@inheritDoc} */
   @Override
   public void add(double yaw, double pitch, double roll)
   {
      this.yaw.add(yaw);
      this.pitch.add(pitch);
      this.roll.add(roll);
   }

   /**
    * Returns the angles of this orientation as an array.
    * 
    * @return in order the angles yaw, pitch, and roll as an array.
    */
   public double[] getYawPitchRoll()
   {
      return new double[] {yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue()};
   }

   /** {@inheritDoc} */
   @Override
   public double getYaw()
   {
      return yaw.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getPitch()
   {
      return pitch.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getRoll()
   {
      return roll.getValue();
   }

   /**
    * Gets the internal reference to the yaw angle.
    *
    * @return the angle around the z-axis as {@code YoVariable}.
    */
   public YoDouble getYoYaw()
   {
      return yaw;
   }

   /**
    * Gets the internal reference to the pitch angle.
    *
    * @return the angle around the y-axis as {@code YoVariable}.
    */
   public YoDouble getYoPitch()
   {
      return pitch;
   }

   /**
    * Gets the internal reference to the roll angle.
    *
    * @return the angle around the x-axis as {@code YoVariable}.
    */
   public YoDouble getYoRoll()
   {
      return roll;
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code orientationOne} to {@code orientationTwo}
    * given the percentage {@code alpha}.
    * <p>
    * The orientations are first transformed into quaternion to then use a <i>Spherical Linear
    * Interpolation</i>.
    * </p>
    *
    * @param orientationOne the first orientation used in the interpolation. Not modified.
    * @param orientationTwo the second orientation used in the interpolation. Not modified.
    * @param alpha          the percentage to use for the interpolation. A value of 0 will result in
    *                       setting this orientation to {@code orientationOne}, while a value of 1 is
    *                       equivalent to setting this orientation to {@code orientationTwo}.
    * @throws ReferenceFrameMismatchException if either {@code orientationOne} or
    *                                         {@code orientationTwo} is not expressed in the same frame
    *                                         as {@code this}.
    */
   public void interpolate(YoFrameYawPitchRoll orientationOne, YoFrameYawPitchRoll orientationTwo, double alpha)
   {
      frameQuaternion.setIncludingFrame(this);
      orientationOne.frameQuaternion.setIncludingFrame(orientationOne);
      orientationTwo.frameQuaternion.setIncludingFrame(orientationTwo);

      frameQuaternion.interpolate(orientationOne.frameQuaternion, orientationTwo.frameQuaternion, alpha);
      set(frameQuaternion);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Attaches a listener to {@code this} that is to be triggered when this orientation components
    * change.
    *
    * @param variableChangedListener the listener to be attached.
    */
   public void attachVariableChangedListener(YoVariableChangedListener variableChangedListener)
   {
      yaw.addListener(variableChangedListener);
      pitch.addListener(variableChangedListener);
      roll.addListener(variableChangedListener);
   }

   /**
    * Creates a copy of {@code this} by finding the duplicated {@code YoVariable}s in the given
    * {@link YoRegistry}.
    * <p>
    * This method does not duplicate {@code YoVariable}s. Assuming the given registry is a duplicate of
    * the registry that was used to create {@code this}, this method searches for the duplicated
    * {@code YoVariable}s and use them to duplicate {@code this}.
    * </p>
    *
    * @param newRegistry YoRegistry to duplicate {@code this} to.
    * @return the duplicate of {@code this}.
    */
   public YoFrameYawPitchRoll duplicate(YoRegistry newRegistry)
   {
      YoDouble yaw = (YoDouble) newRegistry.findVariable(this.yaw.getFullNameString());
      YoDouble pitch = (YoDouble) newRegistry.findVariable(this.pitch.getFullNameString());
      YoDouble roll = (YoDouble) newRegistry.findVariable(this.roll.getFullNameString());
      return new YoFrameYawPitchRoll(yaw, pitch, roll, getReferenceFrame());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameYawPitchRollReadOnly)
         return FixedFrameYawPitchRollBasics.super.equals((FrameYawPitchRollReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.addToHashCode(EuclidHashCodeTools.toLongHashCode(getYaw(), getPitch(), getRoll()), referenceFrame);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameYawPitchRollString(this);
   }
}
