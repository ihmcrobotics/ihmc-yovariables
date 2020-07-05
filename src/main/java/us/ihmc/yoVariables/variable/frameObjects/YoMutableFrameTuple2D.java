package us.ihmc.yoVariables.variable.frameObjects;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public abstract class YoMutableFrameTuple2D extends YoMutableFrameObject implements FrameTuple2DBasics
{
   private final YoDouble x;
   private final YoDouble y;

   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry);
      setToZero(referenceFrame);
   }

   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double x, double y)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, x, y);
   }

   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tupleArray);
   }

   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                Tuple2DReadOnly tuple2DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tuple2DReadOnly);
   }

   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                Tuple3DReadOnly tuple3DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tuple3DReadOnly);
   }

   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple2DReadOnly other)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(other);
   }

   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple3DReadOnly other)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(other);
   }

   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      setReferenceFrame(ReferenceFrame.getWorldFrame());
   }

   public YoMutableFrameTuple2D(YoDouble x, YoDouble y, YoLong frameIndex, FrameIndexMap frameIndexMap)
   {
      super(frameIndex, frameIndexMap);
      this.x = x;
      this.y = y;
   }

   @Override
   public double getX()
   {
      return x.getValue();
   }

   @Override
   public double getY()
   {
      return y.getValue();
   }

   public YoDouble getYoX()
   {
      return x;
   }

   public YoDouble getYoY()
   {
      return y;
   }

   @Override
   public void setX(double x)
   {
      this.x.set(x);
   }

   @Override
   public void setY(double y)
   {
      this.y.set(y);
   }

   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      ReferenceFrame referenceFrame = getReferenceFrame();
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame, false);
      setReferenceFrame(desiredFrame);
   }

   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameTuple2DString(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameTuple2DReadOnly)
         return equals((FrameTuple2DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.toIntHashCode(getX(), getY()), getReferenceFrame());
   }
}