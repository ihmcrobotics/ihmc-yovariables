package us.ihmc.yoVariables.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * {@code FrameVector3DBasics} implementation which components {@code x}, {@code y}, and {@code z}
 * are backed with {@code YoDouble}s.
 */
public class YoMutableFrameVector3D extends YoMutableFrameTuple3D implements FrameVector3DBasics
{
   /**
    * Creates a new {@code YoMutableFrameVector3D}, initializes its components to zero and its
    * reference frame to {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
   }

   /**
    * Creates a new {@code YoMutableFrameVector3D}, initializes its components to zero and its
    * reference frame to {@code referenceFrame}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this vector.
    * @param registry       the registry to register child variables to.
    */
   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame);
   }

   /**
    * Creates a new {@code YoMutableFrameVector3D}, initializes its components and its reference frame.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this vector.
    * @param registry       the registry to register child variables to.
    * @param x              the initial value for the x-component.
    * @param y              the initial value for the y-component.
    * @param z              the initial value for the z-component.
    */
   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double x, double y, double z)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, x, y, z);
   }

   /**
    * Creates a new {@code YoMutableFrameVector3D}, initializes its components and its reference frame.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this vector.
    * @param registry       the registry to register child variables to.
    * @param tupleArray     the array containing this vector's components. Not modified.
    */
   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tupleArray);
   }

   /**
    * Creates a new {@code YoMutableFrameVector3D}, initializes its components and its reference frame.
    *
    * @param namePrefix      a unique name string to use as the prefix for child variable names.
    * @param nameSuffix      a string to use as the suffix for child variable names.
    * @param referenceFrame  the reference frame for this vector.
    * @param registry        the registry to register child variables to.
    * @param tuple2DReadOnly the tuple used to initializes this vector's components. Not modified.
    */
   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple2DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameVector3D}, initializes its components and its reference frame.
    *
    * @param namePrefix      a unique name string to use as the prefix for child variable names.
    * @param nameSuffix      a string to use as the suffix for child variable names.
    * @param referenceFrame  the reference frame for this vector.
    * @param registry        the registry to register child variables to.
    * @param tuple3DReadOnly the tuple used to initializes this vector's components. Not modified.
    */
   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple3DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameVector3D}, initializes its components and its reference frame.
    *
    * @param namePrefix           a unique name string to use as the prefix for child variable names.
    * @param nameSuffix           a string to use as the suffix for child variable names.
    * @param registry             the registry to register child variables to.
    * @param frameTuple2DReadOnly the tuple used to initializes this vector's components and reference
    *                             frame. Not modified.
    */
   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, frameTuple2DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameVector3D}, initializes its components and its reference frame.
    *
    * @param namePrefix           a unique name string to use as the prefix for child variable names.
    * @param nameSuffix           a string to use as the suffix for child variable names.
    * @param registry             the registry to register child variables to.
    * @param frameTuple3DReadOnly the tuple used to initializes this vector's components and reference
    *                             frame. Not modified.
    */
   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, frameTuple3DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameVector3D} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param x             the variable to use for the x-component.
    * @param y             the variable to use for the y-component.
    * @param z             the variable to use for the z-component.
    * @param frameIndex    the variable used to track the current reference frame.
    * @param frameIndexMap the frame index manager used to store and retrieve a reference frame.
    */
   public YoMutableFrameVector3D(YoDouble x, YoDouble y, YoDouble z, YoLong frameIndex, FrameIndexMap frameIndexMap)
   {
      super(x, y, z, frameIndex, frameIndexMap);
   }

   @Override
   public String toString()
   {
      return super.toString() + " (vector)";
   }
}
