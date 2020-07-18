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

public class YoMutableFrameVector3D extends YoMutableFrameTuple3D implements FrameVector3DBasics
{
   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
   }

   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                 Tuple3DReadOnly tuple3DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple3DReadOnly);
   }

   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                 Tuple2DReadOnly tuple2DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple2DReadOnly);
   }

   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double x, double y, double z)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, x, y, z);
   }

   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tupleArray);
   }

   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame);
   }

   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, frameTuple2DReadOnly);
   }

   public YoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, frameTuple3DReadOnly);
   }

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