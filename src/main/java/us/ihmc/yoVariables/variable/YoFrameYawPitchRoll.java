package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameYawPitchRollBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;

/**
 * Defines a 3D representation as {@code YoFrameQuaternion} but using the Euler angles.
 */
public class YoFrameYawPitchRoll implements FixedFrameYawPitchRollBasics
{
   private final YoDouble yaw, pitch, roll; // This is where the data is stored. All operations must act on these numbers.
   private final ReferenceFrame referenceFrame;

   private final FrameQuaternion frameQuaternion = new FrameQuaternion();

   private boolean enableNotifications = true;

   public YoFrameYawPitchRoll(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   public YoFrameYawPitchRoll(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      yaw = new YoDouble(YoFrameVariableNameTools.createName(namePrefix, "yaw", nameSuffix), registry);
      pitch = new YoDouble(YoFrameVariableNameTools.createName(namePrefix, "pitch", nameSuffix), registry);
      roll = new YoDouble(YoFrameVariableNameTools.createName(namePrefix, "roll", nameSuffix), registry);

      this.referenceFrame = referenceFrame;
   }

   public YoFrameYawPitchRoll(YoDouble yaw, YoDouble pitch, YoDouble roll, ReferenceFrame referenceFrame)
   {
      this.yaw = yaw;
      this.pitch = pitch;
      this.roll = roll;

      this.referenceFrame = referenceFrame;
   }

   @Override
   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      setYawPitchRoll(yaw, pitch, roll, true);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll, boolean notifyListeners)
   {
      this.yaw.set(yaw, notifyListeners);
      this.pitch.set(pitch, notifyListeners);
      this.roll.set(roll, notifyListeners);
   }

   @Override
   public void setYaw(double yaw)
   {
      this.yaw.set(yaw, enableNotifications);
   }

   @Override
   public void setPitch(double pitch)
   {
      this.pitch.set(pitch, enableNotifications);
   }

   @Override
   public void setRoll(double roll)
   {
      this.roll.set(roll, enableNotifications);
   }

   public void set(FrameOrientation3DReadOnly frameOrientation, boolean notifyListeners)
   {
      enableNotifications = notifyListeners;
      set(frameOrientation);
      enableNotifications = true;
   }

   public void set(Orientation3DReadOnly orientation, boolean notifyListeners)
   {
      enableNotifications = notifyListeners;
      set(orientation);
      enableNotifications = true;
   }

   public void setMatchingFrame(FrameOrientation3DReadOnly orientation, boolean notifyListeners)
   {
      enableNotifications = notifyListeners;
      setMatchingFrame(orientation);
      enableNotifications = true;
   }

   /**
    * Sets the orientation of this to the origin of the passed in ReferenceFrame.
    *
    * @param referenceFrame
    */
   public void setFromReferenceFrame(ReferenceFrame referenceFrame, boolean notifyListeners)
   {
      enableNotifications = notifyListeners;
      setFromReferenceFrame(referenceFrame);
      enableNotifications = true;
   }

   public void add(YoFrameYawPitchRoll orientation)
   {
      yaw.add(orientation.getYaw());
      pitch.add(orientation.getPitch());
      roll.add(orientation.getRoll());
   }

   @Override
   public void add(double yaw, double pitch, double roll)
   {
      this.yaw.add(yaw);
      this.pitch.add(pitch);
      this.roll.add(roll);
   }

   public double[] getYawPitchRoll()
   {
      return new double[] {yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue()};
   }

   @Override
   public double getYaw()
   {
      return yaw.getValue();
   }

   @Override
   public double getPitch()
   {
      return pitch.getValue();
   }

   @Override
   public double getRoll()
   {
      return roll.getValue();
   }

   public YoDouble getYoYaw()
   {
      return yaw;
   }

   public YoDouble getYoPitch()
   {
      return pitch;
   }

   public YoDouble getYoRoll()
   {
      return roll;
   }

   public void interpolate(YoFrameYawPitchRoll orientationOne, YoFrameYawPitchRoll orientationTwo, double alpha)
   {
      frameQuaternion.setIncludingFrame(this);
      orientationOne.frameQuaternion.setIncludingFrame(orientationOne);
      orientationTwo.frameQuaternion.setIncludingFrame(orientationTwo);

      frameQuaternion.interpolate(orientationOne.frameQuaternion, orientationTwo.frameQuaternion, alpha);
      set(frameQuaternion);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      yaw.addVariableChangedListener(variableChangedListener);
      pitch.addVariableChangedListener(variableChangedListener);
      roll.addVariableChangedListener(variableChangedListener);
   }

   /**
    * Creates a copy of {@code this} by finding the duplicated {@code YoVariable}s in the given
    * {@link YoVariableRegistry}.
    * <p>
    * This method does not duplicate {@code YoVariable}s. Assuming the given registry is a duplicate of
    * the registry that was used to create {@code this}, this method searches for the duplicated
    * {@code YoVariable}s and use them to duplicate {@code this}.
    * </p>
    *
    * @param newRegistry YoVariableRegistry to duplicate {@code this} to.
    * @return the duplicate of {@code this}.
    */
   public YoFrameYawPitchRoll duplicate(YoVariableRegistry newRegistry)
   {
      YoDouble yaw = (YoDouble) newRegistry.findVariable(this.yaw.getFullNameWithNameSpace());
      YoDouble pitch = (YoDouble) newRegistry.findVariable(this.pitch.getFullNameWithNameSpace());
      YoDouble roll = (YoDouble) newRegistry.findVariable(this.roll.getFullNameWithNameSpace());
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
