package us.ihmc.yoVariables.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class YoMutableFramePoint2D extends YoMutableFrameTuple2D implements FramePoint2DBasics
{
   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
   }

   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame);
   }

   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double x, double y)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, x, y);
   }

   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tupleArray);
   }

   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                Tuple2DReadOnly tuple2DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple2DReadOnly);
   }

   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame,
                                Tuple3DReadOnly tuple3DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple3DReadOnly);
   }

   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple2DReadOnly other)
   {
      super(namePrefix, nameSuffix, registry, other);
   }

   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple3DReadOnly other)
   {
      super(namePrefix, nameSuffix, registry, other);
   }

   public YoMutableFramePoint2D(YoDouble x, YoDouble y, YoLong frameIndex, FrameIndexMap frameIndexMap)
   {
      super(x, y, frameIndex, frameIndexMap);
   }

   @Override
   public String toString()
   {
      return super.toString() + " (point)";
   }
}
