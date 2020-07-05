package us.ihmc.yoVariables.variable.frameObjects;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class YoMutableFrameQuaternion extends YoMutableFrameObject implements FrameQuaternionBasics
{
   private final YoDouble qx;
   private final YoDouble qy;
   private final YoDouble qz;
   private final YoDouble qs;

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry);
      setToZero(referenceFrame);
   }

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double x, double y,
                                   double z, double s)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, x, y, z, s);
   }

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tupleArray);
   }

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, DMatrix matrix)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, matrix);
   }

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                   QuaternionReadOnly quaternionReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, quaternionReadOnly);
   }

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                   Tuple4DReadOnly tuple4DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tuple4DReadOnly);
   }

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                   Orientation3DReadOnly orientation3DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, orientation3DReadOnly);
   }

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                   Vector3DReadOnly rotationVector)
   {
      this(namePrefix, nameSuffix, registry);
      setRotationVectorIncludingFrame(referenceFrame, rotationVector);
   }

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double yaw, double pitch,
                                   double roll)
   {
      this(namePrefix, nameSuffix, registry);
      setYawPitchRollIncludingFrame(referenceFrame, yaw, pitch, roll);
   }

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(frameTuple4DReadOnly);
   }

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, FrameQuaternionReadOnly other)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(other);
   }

   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
      qx = new YoDouble(YoFrameVariableNameTools.createQxName(namePrefix, nameSuffix), registry);
      qy = new YoDouble(YoFrameVariableNameTools.createQyName(namePrefix, nameSuffix), registry);
      qz = new YoDouble(YoFrameVariableNameTools.createQzName(namePrefix, nameSuffix), registry);
      qs = new YoDouble(YoFrameVariableNameTools.createQsName(namePrefix, nameSuffix), registry);
      setToZero();
      setReferenceFrame(ReferenceFrame.getWorldFrame());
   }

   public YoMutableFrameQuaternion(YoDouble qx, YoDouble qy, YoDouble qz, YoDouble qs, YoLong frameIndex, FrameIndexMap frameIndexMap)
   {
      super(frameIndex, frameIndexMap);
      this.qx = qx;
      this.qy = qy;
      this.qz = qz;
      this.qs = qs;
   }

   @Override
   public double getX()
   {
      return qx.getValue();
   }

   @Override
   public double getY()
   {
      return qy.getValue();
   }

   @Override
   public double getZ()
   {
      return qz.getValue();
   }

   @Override
   public double getS()
   {
      return qs.getValue();
   }

   public YoDouble getYoX()
   {
      return qx;
   }

   public YoDouble getYoY()
   {
      return qy;
   }

   public YoDouble getYoZ()
   {
      return qz;
   }

   public YoDouble getYoS()
   {
      return qs;
   }

   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      this.qx.set(qx);
      this.qy.set(qy);
      this.qz.set(qz);
      this.qs.set(qs);
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.toIntHashCode(getX(), getY(), getZ(), getS()), getReferenceFrame());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameTuple4DReadOnly)
         return equals((FrameTuple4DReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameTuple4DString(this);
   }
}
