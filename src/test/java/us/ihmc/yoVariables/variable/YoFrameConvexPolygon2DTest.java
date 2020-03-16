package us.ihmc.yoVariables.variable;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoFrameConvexPolygon2DTest
{
   private static final int ITERATIONS = 1000;

   @BeforeAll
   public static void disableStackTrace()
   {
      YoVariable.SAVE_STACK_TRACE = false;
   }

   @Test
   public void testBugIssue6()
   {
      YoFrameConvexPolygon2D polygon = new YoFrameConvexPolygon2D("Test", ReferenceFrame.getWorldFrame(), 20, new YoVariableRegistry("test"));

      Point2DReadOnly point1 = new Point2D(0.111, 0.222);
      Point2DReadOnly point2 = new Point2D(0.222, 0.333);
      polygon.addVertex(point1);
      polygon.addVertex(point2);
      polygon.update();
      assertEquals(2, polygon.getNumberOfVertices());
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(point1, polygon.getVertex(0), 1e-8);
      polygon.removeVertex(0);
      polygon.update();
      assertEquals(1, polygon.getNumberOfVertices());
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(point2, polygon.getVertex(0), 1e-8);
   }

   @Test
   public void testRemoveVertex()
   {
      Random random = new Random(6325);

      for (int i = 0; i < ITERATIONS; i++)
      {
         FrameConvexPolygon2D referencePolygon = EuclidFrameRandomTools.nextFrameConvexPolygon2D(random, ReferenceFrame.getWorldFrame(), 1.0, 60);
         YoFrameConvexPolygon2D testedPolygon = new YoFrameConvexPolygon2D("Test", ReferenceFrame.getWorldFrame(), 20, new YoVariableRegistry("test"));
         testedPolygon.set(referencePolygon);

         int numberOfVerticesToRemove = random.nextInt(referencePolygon.getNumberOfVertices());

         for (int j = 0; j < numberOfVerticesToRemove; j++)
         {
            int indexToRemove = random.nextInt(referencePolygon.getNumberOfVertices());
            referencePolygon.removeVertex(indexToRemove);
            testedPolygon.removeVertex(indexToRemove);
         }

         referencePolygon.update();
         testedPolygon.update();

         assertEquals(referencePolygon.getNumberOfVertices(), testedPolygon.getNumberOfVertices());
         EuclidFrameTestTools.assertFrameConvexPolygon2DEquals(referencePolygon, testedPolygon, 0.0);
      }
   }
}
