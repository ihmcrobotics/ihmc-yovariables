package us.ihmc.yoVariables.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * {@code FramePoint3DBasics} implementation which coordinates {@code x}, {@code y}, and {@code z}
 * are backed with {@code YoDouble}s.
 */
public class YoMutableFramePoint3D extends YoMutableFrameTuple3D implements FramePoint3DBasics
{
   /**
    * Creates a new {@code YoMutableFramePoint3D}, initializes its coordinates to zero and its
    * reference frame to {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
   }

   /**
    * Creates a new {@code YoMutableFramePoint3D}, initializes its coordinates to zero and its
    * reference frame to {@code referenceFrame}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this point.
    * @param registry       the registry to register child variables to.
    */
   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame);
   }

   /**
    * Creates a new {@code YoMutableFramePoint3D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this point.
    * @param registry       the registry to register child variables to.
    * @param x              the initial value for the x-coordinate.
    * @param y              the initial value for the y-coordinate.
    * @param z              the initial value for the z-coordinate.
    */
   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double x, double y, double z)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, x, y, z);
   }

   /**
    * Creates a new {@code YoMutableFramePoint3D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this point.
    * @param registry       the registry to register child variables to.
    * @param tupleArray     the array containing this point's coordinates. Not modified.
    */
   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tupleArray);
   }

   /**
    * Creates a new {@code YoMutableFramePoint3D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix      a unique name string to use as the prefix for child variable names.
    * @param nameSuffix      a string to use as the suffix for child variable names.
    * @param referenceFrame  the reference frame for this point.
    * @param registry        the registry to register child variables to.
    * @param tuple2DReadOnly the tuple used to initializes this point's coordinates. Not modified.
    */
   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple2DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFramePoint3D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix      a unique name string to use as the prefix for child variable names.
    * @param nameSuffix      a string to use as the suffix for child variable names.
    * @param referenceFrame  the reference frame for this point.
    * @param registry        the registry to register child variables to.
    * @param tuple3DReadOnly the tuple used to initializes this point's coordinates. Not modified.
    */
   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple3DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFramePoint3D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix           a unique name string to use as the prefix for child variable names.
    * @param nameSuffix           a string to use as the suffix for child variable names.
    * @param registry             the registry to register child variables to.
    * @param frameTuple2DReadOnly the tuple used to initializes this point's coordinates and reference
    *                             frame. Not modified.
    */
   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, frameTuple2DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFramePoint3D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix           a unique name string to use as the prefix for child variable names.
    * @param nameSuffix           a string to use as the suffix for child variable names.
    * @param registry             the registry to register child variables to.
    * @param frameTuple3DReadOnly the tuple used to initializes this point's coordinates and reference
    *                             frame. Not modified.
    */
   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, frameTuple3DReadOnly);
   }

   /**
    * Creates a new {@code YoFramePoint2D} using the given {@code YoVariable}s and sets its reference
    * frame to {@code referenceFrame}.
    *
    * @param x             the variable to use for the x-coordinate.
    * @param y             the variable to use for the y-coordinate.
    * @param z             the variable to use for the z-coordinate.
    * @param frameIndex    the variable used to track the current reference frame.
    * @param frameIndexMap the frame index manager used to store and retrieve a reference frame.
    */
   public YoMutableFramePoint3D(YoDouble x, YoDouble y, YoDouble z, YoLong frameIndex, FrameIndexMap frameIndexMap)
   {
      super(x, y, z, frameIndex, frameIndexMap);
   }

   @Override
   public String toString()
   {
      return super.toString() + " (point)";
   }
}
