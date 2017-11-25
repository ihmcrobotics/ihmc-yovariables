package us.ihmc.yoVariables.dataBuffer;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class KeyPointsTest
{
   private KeyPoints keyPoints;

   @Before
   public void setup()
   {
      keyPoints = new KeyPoints();
   }

   @After
   public void tearDown()
   {
      keyPoints = null;
   }

   @Test(timeout = 30000)
   public void testSetKeyPoint()
   {
      assertTrue(keyPoints.getPoints().size() == 0);
      assertTrue(keyPoints.setKeyPoint(0));
      assertTrue(keyPoints.getPoints().size() == 1);
      assertTrue(keyPoints.setKeyPoint(4));
      assertTrue(keyPoints.getPoints().size() == 2);
      assertTrue(keyPoints.setKeyPoint(3));
      assertTrue(keyPoints.getPoints().size() == 3);
   }

   @Test(timeout = 30000)
   public void testRemoveDuplicateKeyPoint()
   {
      assertTrue(keyPoints.getPoints().size() == 0);
      assertTrue(keyPoints.setKeyPoint(3));
      assertTrue(keyPoints.getPoints().size() == 1);

      assertFalse(keyPoints.setKeyPoint(3));
      assertTrue(keyPoints.getPoints().size() == 0);
   }

   @Test(timeout = 30000)
   public void testGetNextTime()
   {
      int[] keyPointTimes = new int[]{3,16,20,48,75};

      int nextTimeNotAdded = keyPoints.getNextTime(88);
      assertTrue(nextTimeNotAdded == 88);

      clearAndFillKeyPoints(keyPointTimes);

      int nextTimeInRange = keyPoints.getNextTime(17);
      assertTrue(nextTimeInRange == 20);

      int nextTimeOutOfRange = keyPoints.getNextTime(99);
      assertTrue(nextTimeOutOfRange == 3);
   }

   @Test(timeout = 30000)
   public void getPreviousTime()
   {
      int[] keyPointTimes = new int[]{3,16,20,48,75};

      int previousTimeNotAdded = keyPoints.getPreviousTime(1);
      assertTrue(previousTimeNotAdded == 1);

      clearAndFillKeyPoints(keyPointTimes);

      int previousTimeInRange = keyPoints.getPreviousTime(47);
      assertTrue(previousTimeInRange == 20);

      int previousTimeOutOfRange = keyPoints.getPreviousTime(1);
      assertTrue(previousTimeOutOfRange == 75);
   }

   @Test(timeout = 30000)
   public void testTrim()
   {
      int[] keyPointTimes = new int[]{3,16,20,48,75};

      clearAndFillKeyPoints(keyPointTimes);

      assertTrue(keyPoints.getPoints().size() == keyPointTimes.length);

      keyPoints.trim(17, 47);
      assertTrue(keyPoints.getPoints().size() == 1);
      assertTrue(keyPoints.getNextTime(1) == 20);

      clearAndFillKeyPoints(keyPointTimes);

      keyPoints.trim(47, 17);
      assertTrue(keyPoints.getPoints().size() == 4);
      assertTrue(keyPoints.getPoints().get(0) == 3);
      assertTrue(keyPoints.getPoints().get(1) == 16);
      assertTrue(keyPoints.getPoints().get(2) == 48);
      assertTrue(keyPoints.getPoints().get(3) == 75);
   }

   @Test(timeout = 30000)
   public void testUseKeyPoints()
   {
      assertFalse(keyPoints.useKeyPoints());
      keyPoints.setUseKeyPoints(true);
      assertTrue(keyPoints.useKeyPoints());
      keyPoints.setUseKeyPoints(false);
      assertFalse(keyPoints.useKeyPoints());
   }

   private void clearAndFillKeyPoints(int[] keyPointTimes)
   {
      keyPoints.getPoints().clear();
      for(int keyPointTime : keyPointTimes)
      {
         keyPoints.setKeyPoint(keyPointTime);
      }
   }
}
