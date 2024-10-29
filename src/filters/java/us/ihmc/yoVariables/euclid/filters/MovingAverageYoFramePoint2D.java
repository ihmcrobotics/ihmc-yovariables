package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;

public class MovingAverageYoFramePoint2D extends YoFramePoint2D
{
   private final SimpleMovingAverageFilteredYoVariable x, y;

   public MovingAverageYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, int windowSize, ReferenceFrame referenceFrame)
   {
      this(new SimpleMovingAverageFilteredYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), windowSize, registry),
           new SimpleMovingAverageFilteredYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), windowSize, registry),
           referenceFrame);
   }

   public MovingAverageYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, int windowSize, YoFramePoint2D unfilteredPoint)
   {
      this(new SimpleMovingAverageFilteredYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), windowSize, unfilteredPoint.getYoX(), registry),
           new SimpleMovingAverageFilteredYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), windowSize, unfilteredPoint.getYoY(), registry),
           unfilteredPoint.getReferenceFrame());
   }

   private MovingAverageYoFramePoint2D(SimpleMovingAverageFilteredYoVariable x, SimpleMovingAverageFilteredYoVariable y, ReferenceFrame referenceFrame)
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

   public void update(Point2DReadOnly point2dUnfiltered)
   {
      update(point2dUnfiltered.getX(), point2dUnfiltered.getY());
   }

   public void update(FramePoint2DReadOnly point2dUnfiltered)
   {
      checkReferenceFrameMatch(point2dUnfiltered);
      update((Point2DReadOnly) point2dUnfiltered);
   }

   public void reset()
   {
      x.reset();
      y.reset();
   }
}
