package us.ihmc.yoVariables.variable.frameObjects;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public abstract class YoMutableFrameTuple3D extends YoMutableFrameObject implements FrameTuple3DBasics
{
   private final YoDouble x;
   private final YoDouble y;
   private final YoDouble z;

   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tuple3DReadOnly);
   }

   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tuple2DReadOnly, 0.0);
   }

   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame referenceFrame, double x, double y, double z)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, x, y, z);
   }

   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tupleArray);
   }

   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry);
      setToZero(referenceFrame);
   }

   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(frameTuple2DReadOnly, 0.0);
   }

   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(frameTuple3DReadOnly);
   }

   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      z = new YoDouble(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
      setReferenceFrame(ReferenceFrame.getWorldFrame());
   }

   public YoMutableFrameTuple3D(YoDouble x, YoDouble y, YoDouble z, YoLong frameIndex, FrameIndexMap frameIndexMap)
   {
      super(frameIndex, frameIndexMap);
      this.x = x;
      this.y = y;
      this.z = z;
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

   @Override
   public double getZ()
   {
      return z.getValue();
   }

   public YoDouble getYoX()
   {
      return x;
   }

   public YoDouble getYoY()
   {
      return y;
   }

   public YoDouble getYoZ()
   {
      return z;
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

   @Override
   public void setZ(double z)
   {
      this.z.set(z);
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple3DString(this) + "-" + getReferenceFrame();
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameTuple3DReadOnly)
         return equals((FrameTuple3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, getX());
      bits = EuclidHashCodeTools.addToHashCode(bits, getY());
      bits = EuclidHashCodeTools.addToHashCode(bits, getZ());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }
}
