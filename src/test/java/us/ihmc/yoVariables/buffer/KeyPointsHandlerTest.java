package us.ihmc.yoVariables.buffer;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class KeyPointsHandlerTest
{
   private KeyPointsHandler keyPoints;

   @BeforeEach
   public void setup()
   {
      keyPoints = new KeyPointsHandler();
   }

   @AfterEach
   public void tearDown()
   {
      keyPoints = null;
   }

   @Test // timeout = 30000
   public void testSetKeyPoint()
   {
      assertTrue(keyPoints.getKeyPoints().size() == 0);
      assertTrue(keyPoints.toggleKeyPoint(0));
      assertTrue(keyPoints.getKeyPoints().size() == 1);
      assertTrue(keyPoints.toggleKeyPoint(4));
      assertTrue(keyPoints.getKeyPoints().size() == 2);
      assertTrue(keyPoints.toggleKeyPoint(3));
      assertTrue(keyPoints.getKeyPoints().size() == 3);
   }

   @Test // timeout = 30000
   public void testRemoveDuplicateKeyPoint()
   {
      assertTrue(keyPoints.getKeyPoints().size() == 0);
      assertTrue(keyPoints.toggleKeyPoint(3));
      assertTrue(keyPoints.getKeyPoints().size() == 1);

      assertFalse(keyPoints.toggleKeyPoint(3));
      assertTrue(keyPoints.getKeyPoints().size() == 0);
   }

   @Test // timeout = 30000
   public void testGetNextTime()
   {
      int[] keyPointTimes = new int[] {3, 16, 20, 48, 75};

      int nextTimeNotAdded = keyPoints.getNextKeyPoint(88);
      assertTrue(nextTimeNotAdded == 88);

      clearAndFillKeyPoints(keyPointTimes);

      int nextTimeInRange = keyPoints.getNextKeyPoint(17);
      assertTrue(nextTimeInRange == 20);

      int nextTimeOutOfRange = keyPoints.getNextKeyPoint(99);
      assertTrue(nextTimeOutOfRange == 3);
   }

   @Test // timeout = 30000
   public void getPreviousTime()
   {
      int[] keyPointTimes = new int[] {3, 16, 20, 48, 75};

      int previousTimeNotAdded = keyPoints.getPreviousKeyPoint(1);
      assertTrue(previousTimeNotAdded == 1);

      clearAndFillKeyPoints(keyPointTimes);

      int previousTimeInRange = keyPoints.getPreviousKeyPoint(47);
      assertTrue(previousTimeInRange == 20);

      int previousTimeOutOfRange = keyPoints.getPreviousKeyPoint(1);
      assertTrue(previousTimeOutOfRange == 75);
   }

   @Test // timeout = 30000
   public void testTrim()
   {
      int[] keyPointTimes = new int[] {3, 16, 20, 48, 75};

      clearAndFillKeyPoints(keyPointTimes);

      assertTrue(keyPoints.getKeyPoints().size() == keyPointTimes.length);

      keyPoints.trimKeyPoints(17, 47);
      assertTrue(keyPoints.getKeyPoints().size() == 1);
      assertTrue(keyPoints.getNextKeyPoint(1) == 20);

      clearAndFillKeyPoints(keyPointTimes);

      keyPoints.trimKeyPoints(47, 17);
      assertTrue(keyPoints.getKeyPoints().size() == 4);
      assertTrue(keyPoints.getKeyPoints().get(0) == 3);
      assertTrue(keyPoints.getKeyPoints().get(1) == 16);
      assertTrue(keyPoints.getKeyPoints().get(2) == 48);
      assertTrue(keyPoints.getKeyPoints().get(3) == 75);
   }

   @Test // timeout = 30000
   public void testUseKeyPoints()
   {
      assertFalse(keyPoints.areKeyPointsEnabled());
      keyPoints.enableKeyPoints(true);
      assertTrue(keyPoints.areKeyPointsEnabled());
      keyPoints.enableKeyPoints(false);
      assertFalse(keyPoints.areKeyPointsEnabled());
   }

   private void clearAndFillKeyPoints(int[] keyPointTimes)
   {
      keyPoints.getKeyPoints().clear();
      for (int keyPointTime : keyPointTimes)
      {
         keyPoints.toggleKeyPoint(keyPointTime);
      }
   }
}
