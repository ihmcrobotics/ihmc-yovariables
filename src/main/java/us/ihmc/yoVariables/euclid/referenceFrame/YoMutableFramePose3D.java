package us.ihmc.yoVariables.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * {@code FramePose3DBasics} implementation which position and orientation backed with
 * {@code YoVariable}s.
 */
public class YoMutableFramePose3D implements FramePose3DBasics, YoMutableFrameObject
{
   /** The position part of this pose 3D. */
   private final YoMutableFramePoint3D position;
   /** The orientation part of this pose 3D. */
   private final YoMutableFrameQuaternion orientation;

   private final YoLong frameId;
   private final FrameIndexMap frameIndexMap;

   /**
    * Creates a pose using the provided YoVariables.
    * <p>
    * If holding on to the position and orientation outside this class make sure to always keep their
    * frames consistent or operations on this pose will fail with a
    * {@link ReferenceFrameMismatchException}.
    * </p>
    *
    * @param position    the {@code YoMutableFramePoint3D} to use internally for the position.
    * @param orientation the {@code YoMutableFrameQuaternion} to use internally for the orientation.
    */
   public YoMutableFramePose3D(YoMutableFramePoint3D position, YoMutableFrameQuaternion orientation)
   {
      this.position = position;
      this.orientation = orientation;
      this.frameId = position.getYoFrameIndex();
      this.frameIndexMap = position.getFrameIndexMap();
      checkFrameConsistency();
   }

   /**
    * Creates a new pose and YoVariables in the provided registry.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoMutableFramePose3D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      frameId = new YoLong(YoFrameVariableNameTools.createName(namePrefix, "frame", nameSuffix), registry);
      frameIndexMap = new FrameIndexMap.FrameIndexHashMap();

      YoDouble x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      YoDouble y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      YoDouble z = new YoDouble(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
      position = new YoMutableFramePoint3D(x, y, z, getYoFrameIndex(), getFrameIndexMap());

      YoDouble qx = new YoDouble(YoFrameVariableNameTools.createQxName(namePrefix, nameSuffix), registry);
      YoDouble qy = new YoDouble(YoFrameVariableNameTools.createQyName(namePrefix, nameSuffix), registry);
      YoDouble qz = new YoDouble(YoFrameVariableNameTools.createQzName(namePrefix, nameSuffix), registry);
      YoDouble qs = new YoDouble(YoFrameVariableNameTools.createQsName(namePrefix, nameSuffix), registry);
      orientation = new YoMutableFrameQuaternion(qx, qy, qz, qs, getYoFrameIndex(), getFrameIndexMap());
      setReferenceFrame(ReferenceFrame.getWorldFrame());
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getPosition()
   {
      checkFrameConsistency();
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameQuaternionBasics getOrientation()
   {
      checkFrameConsistency();
      return orientation;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      checkFrameConsistency();
      return YoMutableFrameObject.super.getReferenceFrame();
   }

   /** {@inheritDoc} */
   @Override
   public YoLong getYoFrameIndex()
   {
      return frameId;
   }

   /** {@inheritDoc} */
   @Override
   public FrameIndexMap getFrameIndexMap()
   {
      return frameIndexMap;
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      YoMutableFrameObject.super.setReferenceFrame(referenceFrame);
      // When constructing this with two YoMutableFramePoint3D objects the position part is updated only by the super implementation.
      orientation.setReferenceFrame(referenceFrame);
   }

   /**
    * This is a check that should be called every time this object is interacted with.
    * <p>
    * If this fails it likely means that you created this pose using
    * {@link #YoMutableFramePose3D(YoMutableFramePoint3D, YoMutableFrameQuaternion)} and changed the
    * reference frame of one of the passed objects without modifying the other one from outside this
    * class. This will make the data structure in here inconsistent.
    * </p>
    */
   private void checkFrameConsistency()
   {
      if (position.getReferenceFrame() == null)
      {
         if (orientation.getReferenceFrame() != null)
            orientation.getReferenceFrame().checkReferenceFrameMatch(position.getReferenceFrame());
      }
      else
      {
         position.checkReferenceFrameMatch(orientation);
      }
   }

   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFramePose3DString(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FramePose3DReadOnly)
         return equals((FramePose3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getPosition(), getOrientation());
   }
}
