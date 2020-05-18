package us.ihmc.yoVariables.variable.frameObjects;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoMutableFramePose3D extends YoMutableFrameObject implements FramePose3DBasics
{
   /** The position part of this pose 3D. */
   private final YoMutableFramePoint3D position;
   /** The orientation part of this pose 3D. */
   private final YoMutableFrameQuaternion orientation;

   /**
    * Creates a pose using the provided YoVariables. If holding on to the position and orientation
    * outside this class make sure to always keep their frames consistent or operations on this pose
    * will fail with a {@link ReferenceFrameMismatchException}.
    *
    * @param position    part of this pose 3D.
    * @param orientation part of this pose 3D.
    */
   public YoMutableFramePose3D(YoMutableFramePoint3D position, YoMutableFrameQuaternion orientation)
   {
      super(position.getYoFrameIndex(), position.getFrameIndexMap());
      this.position = position;
      this.orientation = orientation;
      checkFrameConsistency();
   }

   /**
    * Creates a new pose and YoVariables in the provided registry.
    *
    * @param namePrefix the prefix given to all YoVariables
    * @param nameSuffix the suffix given to all YoVariables
    * @param registry   is where the Yovariables backing this object are created.
    */
   public YoMutableFramePose3D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);

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

   @Override
   public FixedFramePoint3DBasics getPosition()
   {
      checkFrameConsistency();
      return position;
   }

   @Override
   public FixedFrameQuaternionBasics getOrientation()
   {
      checkFrameConsistency();
      return orientation;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      checkFrameConsistency();
      return super.getReferenceFrame();
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      super.setReferenceFrame(referenceFrame);
      // When constructing this with two YoMutableFramePoint3D objects the position part is updated only by the super implementation.
      orientation.setReferenceFrame(referenceFrame);
   }

   /**
    * This is a check that should be called every time this object is interacted with. If this failes
    * it likely means that you created this pose using
    * {@link #YoMutableFramePose3D(YoMutableFramePoint3D, YoMutableFrameQuaternion)} and changed the
    * reference frame of one of the passed objects without modifying the other one from outside this
    * class. This will make the data structure in here inconsistent.
    */
   private void checkFrameConsistency()
   {
      position.checkReferenceFrameMatch(orientation);
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
