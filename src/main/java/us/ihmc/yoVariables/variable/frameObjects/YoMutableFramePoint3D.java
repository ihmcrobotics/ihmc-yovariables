package us.ihmc.yoVariables.variable.frameObjects;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class YoMutableFramePoint3D extends YoMutableFrameTuple3D implements FramePoint3DBasics
{
   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
   }

   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame referenceFrame,
                                Tuple3DReadOnly tuple3DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple3DReadOnly);
   }

   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame referenceFrame,
                                Tuple2DReadOnly tuple2DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple2DReadOnly);
   }

   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame referenceFrame, double x, double y, double z)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, x, y, z);
   }

   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tupleArray);
   }

   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame);
   }

   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, frameTuple2DReadOnly);
   }

   public YoMutableFramePoint3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, frameTuple3DReadOnly);
   }

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
