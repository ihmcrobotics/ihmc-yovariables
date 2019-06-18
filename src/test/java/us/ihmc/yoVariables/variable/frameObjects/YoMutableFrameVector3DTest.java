package us.ihmc.yoVariables.variable.frameObjects;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameTuple3DBasicsTest;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoMutableFrameVector3DTest extends FrameTuple3DBasicsTest<YoMutableFrameVector3D>
{
   static
   {
      YoVariable.SAVE_STACK_TRACE = false;
      YoVariable.warnAboutNullRegistries = false;
   }

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public Tuple3DBasics createRandomFramelessTuple(Random random)
   {
      return EuclidCoreRandomTools.nextVector3D(random);
   }

   @Override
   public YoMutableFrameVector3D createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return new YoMutableFrameVector3D("", "", null, referenceFrame, x, y, z);
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test YoMutableFrameVector3D()
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == worldFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(YoMutableFrameVector3D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomFrame);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(YoMutableFrameVector3D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(ReferenceFrame referenceFrame, double x, double y, double z)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.nextVector3D(random);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomFrame, randomTuple.getX(), randomTuple.getY(), randomTuple.getZ());
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, YoMutableFrameVector3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.nextVector3D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomFrame, array);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, YoMutableFrameVector3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.nextVector3D(random);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomFrame, randomTuple);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, YoMutableFrameVector3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector2D randomTuple2D = EuclidCoreRandomTools.nextVector2D(random);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomFrame, randomTuple2D);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple2D, new Vector2D(YoMutableFrameVector3D), EPSILON);
         assertTrue(YoMutableFrameVector3D.getZ() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector2D randomFrameTuple2D = EuclidFrameRandomTools.nextFrameVector2D(random, randomFrame);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomFrameTuple2D);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomFrameTuple2D, new Vector2D(YoMutableFrameVector3D), EPSILON);
         assertTrue(YoMutableFrameVector3D.getZ() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(FrameTuple3DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameTuple3DReadOnly randomTuple = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomTuple);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, YoMutableFrameVector3D, EPSILON);
         EuclidFrameTestTools.assertFrameTuple3DEquals(randomTuple, YoMutableFrameVector3D, EPSILON);
      }
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple3DReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameTuple3DReadOnly source = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         YoMutableFrameVector3D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source);

         YoMutableFrameVector3D expected = new YoMutableFrameVector3D("", "", null, source);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertFrameTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple2DReadOnly other, double z)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameTuple2DReadOnly source = EuclidFrameRandomTools.nextFrameVector2D(random, sourceFrame);
         double z = EuclidCoreRandomTools.nextDouble(random);
         YoMutableFrameVector3D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source, z);

         YoMutableFrameVector3D expected = new YoMutableFrameVector3D("", "", null);
         expected.setIncludingFrame(source, z);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertFrameTuple3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      Random random = new Random(43563);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random);
         YoMutableFrameVector3D actual = new YoMutableFrameVector3D("", "", null, initialFrame, expected);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expected.applyTransform(transform);

         actual.changeFrame(anotherFrame);
         assertTrue(anotherFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);

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
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoMutableFrameVector3D frameVector1 = new YoMutableFrameVector3D("", "", null, worldFrame, EuclidCoreRandomTools.nextVector3D(random));
         YoMutableFrameVector3D frameVector2 = new YoMutableFrameVector3D("", "", null, worldFrame);
         double epsilon = random.nextDouble();
         Vector3D difference;

         difference = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon);
         frameVector2.add(frameVector1, difference);
         assertTrue(frameVector1.geometricallyEquals(frameVector2, epsilon));

         difference = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon);
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
         Vector3D expected = EuclidCoreRandomTools.nextVector3D(random, -1.0e15, 1.0e15);
         YoMutableFrameVector3D actual = new YoMutableFrameVector3D("", "", null, worldFrame, expected);

         assertEquals(expected.hashCode(), actual.hashCode());
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Vector3D.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Vector3D.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Vector3D.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(YoMutableFrameVector3D.class, Vector3D.class, true, 1, framelessMethodsToIgnore);
   }
}
