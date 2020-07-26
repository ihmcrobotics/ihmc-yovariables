package us.ihmc.yoVariables.euclid.referenceFrame;

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
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * {@code FrameQuaternionBasics} implementation which components {@code x}, {@code y}, {@code z},
 * {@code s} are backed with {@code YoDouble}s.
 */
public class YoMutableFrameQuaternion extends YoQuaternion implements FrameQuaternionBasics, YoMutableFrameObject
{
   private final YoLong frameId;
   private final FrameIndexMap frameIndexMap;

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes it to the neutral quaternion and its
    * reference frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param registry       the registry to register child variables to.
    * @param referenceFrame the reference frame for this {@code YoMutableFrameQuaternion}.
    * @see #setToZero(ReferenceFrame)
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry);
      setToZero(referenceFrame);
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes its components and its reference
    * frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param registry       the registry to register child variables to.
    * @param referenceFrame the reference frame for this {@code YoMutableFrameQuaternion}.
    * @param x              the initial value for the x-component.
    * @param y              the initial value for the y-component.
    * @param z              the initial value for the z-component.
    * @param s              the initial value for the s-component.
    * @see #setIncludingFrame(ReferenceFrame, double, double, double, double)
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double x, double y, double z,
                                   double s)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, x, y, z, s);
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes its components and its reference
    * frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param registry       the registry to register child variables to.
    * @param referenceFrame the reference frame for this {@code YoMutableFrameQuaternion}.
    * @param tupleArray     the array containing the initial values for this quaternion's x, y, z, and
    *                       s components in order. Not modified.
    * @see #setIncludingFrame(ReferenceFrame, double[])
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tupleArray);
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes its components and its reference
    * frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param registry       the registry to register child variables to.
    * @param referenceFrame the reference frame for this {@code YoMutableFrameQuaternion}.
    * @param matrix         the column vector containing the new values for this quaternion's
    *                       components. Not modified.
    * @see #setIncludingFrame(ReferenceFrame, DMatrix)
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, DMatrix matrix)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, matrix);
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes its components and its reference
    * frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix         a unique name string to use as the prefix for child variable names.
    * @param nameSuffix         a string to use as the suffix for child variable names.
    * @param registry           the registry to register child variables to.
    * @param referenceFrame     the reference frame for this {@code YoMutableFrameQuaternion}.
    * @param quaternionReadOnly the quaternion used to initialize {@code this}. Not modified.
    * @see #setIncludingFrame(ReferenceFrame, QuaternionReadOnly)
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                   QuaternionReadOnly quaternionReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, quaternionReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes its components and its reference
    * frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix      a unique name string to use as the prefix for child variable names.
    * @param nameSuffix      a string to use as the suffix for child variable names.
    * @param registry        the registry to register child variables to.
    * @param referenceFrame  the reference frame for this {@code YoMutableFrameQuaternion}.
    * @param tuple4DReadOnly tuple used to initialize this quaternion. Not Modified.
    * @see #setIncludingFrame(ReferenceFrame, Tuple4DReadOnly)
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tuple4DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes its components and its reference
    * frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix            a unique name string to use as the prefix for child variable names.
    * @param nameSuffix            a string to use as the suffix for child variable names.
    * @param registry              the registry to register child variables to.
    * @param referenceFrame        the reference frame for this {@code YoMutableFrameQuaternion}.
    * @param orientation3DReadOnly the orientation used to initialize {@code this}. Not modified.
    * @see #setIncludingFrame(ReferenceFrame, Orientation3DReadOnly)
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                   Orientation3DReadOnly orientation3DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, orientation3DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes its components and its reference
    * frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param registry       the registry to register child variables to.
    * @param referenceFrame the reference frame for this {@code YoMutableFrameQuaternion}.
    * @param rotationVector rotation vector used to initialize this quaternion. Not Modified.
    * @see #setIncludingFrame(ReferenceFrame, Vector3DReadOnly)
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
   {
      this(namePrefix, nameSuffix, registry);
      setRotationVectorIncludingFrame(referenceFrame, rotationVector);
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes its components and its reference
    * frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param registry       the registry to register child variables to.
    * @param referenceFrame the reference frame for this {@code YoMutableFrameQuaternion}.
    * @param yaw            the angle to rotate about the z-axis.
    * @param pitch          the angle to rotate about the y-axis.
    * @param roll           the angle to rotate about the x-axis.
    * @see #setYawPitchRollIncludingFrame(ReferenceFrame, double, double, double)
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double yaw, double pitch,
                                   double roll)
   {
      this(namePrefix, nameSuffix, registry);
      setYawPitchRollIncludingFrame(referenceFrame, yaw, pitch, roll);
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes its components and its reference
    * frame, and registers variables to {@code registry}.
    *
    * @param namePrefix           a unique name string to use as the prefix for child variable names.
    * @param nameSuffix           a string to use as the suffix for child variable names.
    * @param registry             the registry to register child variables to.
    * @param frameTuple4DReadOnly tuple used to initialize this quaternion. Not Modified.
    * @see #setIncludingFrame(FrameTuple4DReadOnly)
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(frameTuple4DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes its components and its reference
    * frame, and registers variables to {@code registry}.
    *
    * @param namePrefix              a unique name string to use as the prefix for child variable
    *                                names.
    * @param nameSuffix              a string to use as the suffix for child variable names.
    * @param registry                the registry to register child variables to.
    * @param frameQuaternionReadOnly the quaternion used to initialize {@code this}. Not Modified.
    * @see #setIncludingFrame(FrameQuaternionReadOnly)
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry, FrameQuaternionReadOnly frameQuaternionReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(frameQuaternionReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion}, initializes it to the neutral quaternion and its
    * reference frame to {@code ReferenceFrame.getWorldFrame()}, and registers variables to
    * {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoMutableFrameQuaternion(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
      frameId = new YoLong(YoGeometryNameTools.assembleName(namePrefix, "frame", nameSuffix), registry);
      frameIndexMap = new FrameIndexMap.FrameIndexHashMap();
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new {@code YoMutableFrameQuaternion} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param qx            an existing variable representing the x value of this
    *                      {@code YoMutableFrameQuaternion}.
    * @param qy            an existing variable representing the y value of this
    *                      {@code YoMutableFrameQuaternion}.
    * @param qz            an existing variable representing the z value of this
    *                      {@code YoMutableFrameQuaternion}.
    * @param qs            an existing variable representing the s value of this
    *                      {@code YoMutableFrameQuaternion}.
    * @param frameIndex    the variable used to track the current reference frame.
    * @param frameIndexMap the frame index manager used to store and retrieve a reference frame.
    */
   public YoMutableFrameQuaternion(YoDouble qx, YoDouble qy, YoDouble qz, YoDouble qs, YoLong frameIndex, FrameIndexMap frameIndexMap)
   {
      super(qx, qy, qz, qs);
      frameId = frameIndex;
      this.frameIndexMap = frameIndexMap;
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      YoMutableFrameObject.super.setReferenceFrame(referenceFrame);
   }

   @Override
   public FrameIndexMap getFrameIndexMap()
   {
      return frameIndexMap;
   }

   @Override
   public YoLong getYoFrameIndex()
   {
      return frameId;
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
