package us.ihmc.yoVariables.variable.frameObjects;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameTuple2DBasicsTest;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoMutableFrameVector2DTest extends FrameTuple2DBasicsTest<YoMutableFrameVector2D>
{
   static
   {
      YoVariable.SAVE_STACK_TRACE = false;
      YoVariable.warnAboutNullRegistries = false;
   }

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public Tuple2DBasics createRandomFramelessTuple(Random random)
   {
      return EuclidCoreRandomTools.nextVector2D(random);
   }

   @Override
   public YoMutableFrameVector2D createFrameTuple(ReferenceFrame referenceFrame, double x, double y)
   {
      return new YoMutableFrameVector2D("", "", null, referenceFrame, x, y);
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test YoMutableFrameVector2D()
         YoMutableFrameVector2D YoMutableFrameVector2D = new YoMutableFrameVector2D("", "", null);
         assertTrue(YoMutableFrameVector2D.getReferenceFrame() == worldFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(YoMutableFrameVector2D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector2D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YoMutableFrameVector2D YoMutableFrameVector2D = new YoMutableFrameVector2D("", "", null, randomFrame);
         assertTrue(YoMutableFrameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(YoMutableFrameVector2D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector2D(ReferenceFrame referenceFrame, double x, double y)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector2D randomTuple = EuclidCoreRandomTools.nextVector2D(random);
         YoMutableFrameVector2D YoMutableFrameVector2D = new YoMutableFrameVector2D("", "", null, randomFrame, randomTuple.getX(), randomTuple.getY());
         assertTrue(YoMutableFrameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, YoMutableFrameVector2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector2D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector2D randomTuple = EuclidCoreRandomTools.nextVector2D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         YoMutableFrameVector2D YoMutableFrameVector2D = new YoMutableFrameVector2D("", "", null, randomFrame, array);
         assertTrue(YoMutableFrameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, YoMutableFrameVector2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector2D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.nextVector3D(random);
         YoMutableFrameVector2D YoMutableFrameVector2D = new YoMutableFrameVector2D("", "", null, randomFrame, randomTuple);
         assertTrue(YoMutableFrameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(new Vector2D(randomTuple), YoMutableFrameVector2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector2D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector2D randomTuple2D = EuclidCoreRandomTools.nextVector2D(random);
         YoMutableFrameVector2D YoMutableFrameVector2D = new YoMutableFrameVector2D("", "", null, randomFrame, randomTuple2D);
         assertTrue(YoMutableFrameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple2D, YoMutableFrameVector2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector2D(FrameTuple2DReadOnly frameTuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector2DReadOnly randomFrameTuple2D = EuclidFrameRandomTools.nextFrameVector2D(random, randomFrame);
         YoMutableFrameVector2D YoMutableFrameVector2D = new YoMutableFrameVector2D("", "", null, randomFrameTuple2D);
         assertTrue(YoMutableFrameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomFrameTuple2D, YoMutableFrameVector2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector2D(FrameTuple3DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D randomTuple = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         YoMutableFrameVector2D YoMutableFrameVector2D = new YoMutableFrameVector2D("", "", null, randomTuple);
         assertTrue(YoMutableFrameVector2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(new Vector2D(randomTuple), YoMutableFrameVector2D, EPSILON);
      }
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple2DReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameTuple2DReadOnly source = EuclidFrameRandomTools.nextFramePoint2D(random, sourceFrame);
         YoMutableFrameVector2D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source);

         YoMutableFrameVector2D expected = new YoMutableFrameVector2D("", "", null, source);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertFrameTuple2DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      Random random = new Random(43563);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random, true);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         YoMutableFrameVector2D actual = new YoMutableFrameVector2D("", "", null, initialFrame, expected);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expected.applyTransform(transform);

         actual.changeFrame(anotherFrame);
         assertTrue(anotherFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);

         ReferenceFrame differentRootFrame = ReferenceFrameTools.constructARootFrame("anotherRootFrame");
         try
         {
            actual.changeFrame(differentRootFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
      }
   }

   @Test
   public void testChangeFrameAndProjectToXYPlane() throws Exception
   {
      Random random = new Random(345345);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame initialFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random);
         YoMutableFrameVector2D actual = new YoMutableFrameVector2D("", "", null, initialFrame, expected);

         expected.applyTransform(initialFrame.getTransformToDesiredFrame(newFrame), false);
         actual.changeFrameAndProjectToXYPlane(newFrame);

         assertTrue(newFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoMutableFrameVector2D frameVector1 = createRandomFrameTuple(random, worldFrame);
         YoMutableFrameVector2D frameVector2 = new YoMutableFrameVector2D("", "", null, worldFrame);
         double epsilon = random.nextDouble();
         Vector2D difference;

         difference = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.99 * epsilon);
         frameVector2.add(frameVector1, difference);
         assertTrue(frameVector1.geometricallyEquals(frameVector2, epsilon));

         difference = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.01 * epsilon);
         frameVector2.add(frameVector1, difference);
         assertFalse(frameVector1.geometricallyEquals(frameVector2, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(763);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D expected = EuclidCoreRandomTools.nextVector2D(random, -1.0e15, 1.0e15);
         YoMutableFrameVector2D actual = new YoMutableFrameVector2D("", "", null, worldFrame, expected);

         assertEquals(expected.hashCode(), actual.hashCode());
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Vector2D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Vector2D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Vector2D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(YoMutableFrameVector2D.class, Vector2D.class, true, 1, framelessMethodsToIgnore);
   }
}

