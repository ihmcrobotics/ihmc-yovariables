package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;

public class MovingAverageYoFrameVector2D extends YoFrameVector2D
{
   private final SimpleMovingAverageFilteredYoVariable x, y;

   public MovingAverageYoFrameVector2D(String namePrefix, String nameSuffix, YoRegistry registry, int windowSize, ReferenceFrame referenceFrame)
   {
      this(new SimpleMovingAverageFilteredYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), windowSize, registry),
           new SimpleMovingAverageFilteredYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), windowSize, registry),
           referenceFrame);
   }

   public MovingAverageYoFrameVector2D(String namePrefix, String nameSuffix, YoRegistry registry, int windowSize, YoFrameVector2D unfilteredVector)
   {
      this(new SimpleMovingAverageFilteredYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), windowSize, unfilteredVector.getYoX(), registry),
           new SimpleMovingAverageFilteredYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), windowSize, unfilteredVector.getYoY(), registry),
           unfilteredVector.getReferenceFrame());
   }

   private MovingAverageYoFrameVector2D(SimpleMovingAverageFilteredYoVariable x, SimpleMovingAverageFilteredYoVariable y, ReferenceFrame referenceFrame)
   {
      super(x, y, referenceFrame);

      this.x = x;
      this.y = y;
   }

   public void update()
   {
      x.update();
      y.update();
   }

   public void update(double xUnfiltered, double yUnfiltered)
   {
      x.update(xUnfiltered);
      y.update(yUnfiltered);
   }

   public void update(Vector2DReadOnly vector2dUnfiltered)
   {
      update(vector2dUnfiltered.getX(), vector2dUnfiltered.getY());
   }

   public void update(FrameVector2DReadOnly vector2dUnfiltered)
   {
      checkReferenceFrameMatch(vector2dUnfiltered);
      update((Vector2DReadOnly) vector2dUnfiltered);
   }

   public void reset()
   {
      x.reset();
      y.reset();
   }
}
