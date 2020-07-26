package us.ihmc.yoVariables.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.YoTuple2D;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * {@code FrameTuple2DBasics} abstract implementation backed with {@code YoDouble}s.
 */
public abstract class YoMutableFrameTuple2D extends YoTuple2D implements FrameTuple2DBasics, YoMutableFrameObject
{
   private final YoLong frameId;
   private final FrameIndexMap frameIndexMap;

   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new {@code YoMutableFrameTuple2D}, initializes its components to zero and its reference
    * frame to {@code referenceFrame}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    */
   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry);
      setToZero(referenceFrame);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple2D}, initializes its components and its reference frame.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    * @param x              the initial value for the x-component.
    * @param y              the initial value for the y-component.
    */
   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double x, double y)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, x, y);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple2D}, initializes its components and its reference frame.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    * @param tupleArray     the array containing this tuple's components. Not modified.
    */
   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tupleArray);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple2D}, initializes its components and its reference frame.
    *
    * @param namePrefix      a unique name string to use as the prefix for child variable names.
    * @param nameSuffix      a string to use as the suffix for child variable names.
    * @param referenceFrame  the reference frame for this tuple.
    * @param registry        the registry to register child variables to.
    * @param tuple2DReadOnly the tuple used to initializes this tuple's components. Not modified.
    */
   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tuple2DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple2D}, initializes its components and its reference frame.
    *
    * @param namePrefix      a unique name string to use as the prefix for child variable names.
    * @param nameSuffix      a string to use as the suffix for child variable names.
    * @param referenceFrame  the reference frame for this tuple.
    * @param registry        the registry to register child variables to.
    * @param tuple3DReadOnly the tuple used to initializes this tuple's components. Not modified.
    */
   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tuple3DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple2D}, initializes its components and its reference frame.
    *
    * @param namePrefix           a unique name string to use as the prefix for child variable names.
    * @param nameSuffix           a string to use as the suffix for child variable names.
    * @param registry             the registry to register child variables to.
    * @param frameTuple2DReadOnly the tuple used to initializes this tuple's components and reference
    *                             frame. Not modified.
    */
   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(frameTuple2DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple2D}, initializes its components and its reference frame.
    *
    * @param namePrefix           a unique name string to use as the prefix for child variable names.
    * @param nameSuffix           a string to use as the suffix for child variable names.
    * @param registry             the registry to register child variables to.
    * @param frameTuple3DReadOnly the tuple used to initializes this tuple's components and reference
    *                             frame. Not modified.
    */
   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(frameTuple3DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple2D}, initializes its components to zero and its reference
    * frame to {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoMutableFrameTuple2D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
      frameId = new YoLong(YoGeometryNameTools.assembleName(namePrefix, "frame", nameSuffix), registry);
      frameIndexMap = new FrameIndexMap.FrameIndexHashMap();
      setReferenceFrame(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new {@code YoMutableFrameTuple2D} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param x             the variable to use for the x-coordinate.
    * @param y             the variable to use for the y-coordinate.
    * @param frameIndex    the variable used to track the current reference frame.
    * @param frameIndexMap the frame index manager used to store and retrieve a reference frame.
    */
   public YoMutableFrameTuple2D(YoDouble x, YoDouble y, YoLong frameIndex, FrameIndexMap frameIndexMap)
   {
      super(x, y);
      this.frameId = frameIndex;
      this.frameIndexMap = frameIndexMap;
   }

   @Override
   public YoLong getYoFrameIndex()
   {
      return frameId;
   }

   @Override
   public FrameIndexMap getFrameIndexMap()
   {
      return frameIndexMap;
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      YoMutableFrameObject.super.setReferenceFrame(referenceFrame);
   }

   /**
    * Performs a transformation of the tuple such that it is expressed in a new frame
    * {@code desireFrame}.
    * <p>
    * Because the transformation between two reference frames is a 3D transformation, the result of
    * transforming this tuple 2D can result in a tuple 3D. This method projects the result of the
    * transformation onto the XY-plane.
    * </p>
    *
    * @param desiredFrame the reference frame in which the tuple is to be expressed.
    */
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