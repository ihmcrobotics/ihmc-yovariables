package us.ihmc.yoVariables.variable.frameObjects;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3DBasicsTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFramePoint3D;

public class YoMutableFramePoint3DTest extends FrameTuple3DBasicsTest<YoMutableFramePoint3D>
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public Tuple3DBasics createRandomFramelessTuple(Random random)
   {
      return EuclidCoreRandomTools.nextPoint3D(random);
   }

   @Override
   public YoMutableFramePoint3D createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      return new YoMutableFramePoint3D("", "", null, referenceFrame, x, y, z);
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test YoMutableFramePoint3D()
         YoMutableFramePoint3D YoMutableFramePoint3D = new YoMutableFramePoint3D("", "", null);
         assertTrue(YoMutableFramePoint3D.getReferenceFrame() == worldFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(YoMutableFramePoint3D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint3D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YoMutableFramePoint3D YoMutableFramePoint3D = new YoMutableFramePoint3D("", "", null, randomFrame);
         assertTrue(YoMutableFramePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DIsSetToZero(YoMutableFramePoint3D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint3D(ReferenceFrame referenceFrame, double x, double y, double z)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.nextPoint3D(random);
         YoMutableFramePoint3D YoMutableFramePoint3D = new YoMutableFramePoint3D("",
                                                                                 "",
                                                                                 null,
                                                                                 randomFrame,
                                                                                 randomTuple.getX(),
                                                                                 randomTuple.getY(),
                                                                                 randomTuple.getZ());
         assertTrue(YoMutableFramePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, YoMutableFramePoint3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint3D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.nextPoint3D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         YoMutableFramePoint3D YoMutableFramePoint3D = new YoMutableFramePoint3D("", "", null, randomFrame, array);
         assertTrue(YoMutableFramePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, YoMutableFramePoint3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point3D randomTuple = EuclidCoreRandomTools.nextPoint3D(random);
         YoMutableFramePoint3D YoMutableFramePoint3D = new YoMutableFramePoint3D("", "", null, randomFrame, randomTuple);
         assertTrue(YoMutableFramePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, YoMutableFramePoint3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint3D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point2D randomTuple2D = EuclidCoreRandomTools.nextPoint2D(random);
         YoMutableFramePoint3D YoMutableFramePoint3D = new YoMutableFramePoint3D("", "", null, randomFrame, randomTuple2D);
         assertTrue(YoMutableFramePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple2D, new Point2D(YoMutableFramePoint3D), EPSILON);
         assertTrue(YoMutableFramePoint3D.getZ() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint2D randomFrameTuple2D = EuclidFrameRandomTools.nextFramePoint2D(random, randomFrame);
         YoMutableFramePoint3D YoMutableFramePoint3D = new YoMutableFramePoint3D("", "", null, randomFrameTuple2D);
         assertTrue(YoMutableFramePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomFrameTuple2D, new Point2D(YoMutableFramePoint3D), EPSILON);
         assertTrue(YoMutableFramePoint3D.getZ() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint3D(FrameTuple3DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameTuple3DReadOnly randomTuple = EuclidFrameRandomTools.nextFramePoint3D(random, randomFrame);
         YoMutableFramePoint3D YoMutableFramePoint3D = new YoMutableFramePoint3D("", "", null, randomTuple);
         assertTrue(YoMutableFramePoint3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomTuple, YoMutableFramePoint3D, EPSILON);
         EuclidFrameTestTools.assertFrameTuple3DEquals(randomTuple, YoMutableFramePoint3D, EPSILON);
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

         FrameTuple3DReadOnly source = EuclidFrameRandomTools.nextFramePoint3D(random, sourceFrame);
         YoMutableFramePoint3D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source);

         YoMutableFramePoint3D expected = new YoMutableFramePoint3D("", "", null, source);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertFrameTuple3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple2DReadOnly other, double z)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameTuple2DReadOnly source = EuclidFrameRandomTools.nextFramePoint2D(random, sourceFrame);
         double z = EuclidCoreRandomTools.nextDouble(random);
         YoMutableFramePoint3D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source, z);

         YoMutableFramePoint3D expected = new YoMutableFramePoint3D("", "", null);
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

         Point3D expectedPoint = EuclidCoreRandomTools.nextPoint3D(random);
         YoMutableFramePoint3D framePoint = new YoMutableFramePoint3D("", "", null, initialFrame, expectedPoint);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expectedPoint.applyTransform(transform);

         framePoint.changeFrame(anotherFrame);
         assertTrue(anotherFrame == framePoint.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, framePoint, 10.0 * EPSILON);

         ReferenceFrame differentRootFrame = ReferenceFrameTools.constructARootFrame("anotherRootFrame");
         try
         {
            framePoint.changeFrame(differentRootFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
      }
   }

   @Test
   public void testSetFromReferenceFrame() throws Exception
   {
      Random random = new Random(6572);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         YoMutableFramePoint3D expected = createEmptyFrameTuple(anotherFrame);
         expected.changeFrame(initialFrame);

         YoMutableFramePoint3D actual = createRandomFrameTuple(random, initialFrame);
         actual.setFromReferenceFrame(anotherFrame);
         assertTrue(initialFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoMutableFramePoint3D framePoint1 = new YoMutableFramePoint3D("", "", null, worldFrame, EuclidCoreRandomTools.nextPoint3D(random));
         YoMutableFramePoint3D framePoint2 = new YoMutableFramePoint3D("", "", null, worldFrame);
         double epsilon = random.nextDouble();
         Vector3D difference;

         difference = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon);
         framePoint2.add(framePoint1, difference);
         assertTrue(framePoint1.geometricallyEquals(framePoint2, epsilon));

         difference = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon);
         framePoint2.add(framePoint1, difference);
         assertFalse(framePoint1.geometricallyEquals(framePoint2, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(763);

      for (int i = 0; i < ITERATIONS; i++)
      {
         FramePoint3D expected = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, -1.0e15, 1.0e15);
         YoMutableFramePoint3D actual = new YoMutableFramePoint3D("", "", null, worldFrame, expected);

         assertEquals(expected.hashCode(), actual.hashCode());
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", Point3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Point3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Point3D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(YoMutableFramePoint3D.class, Point3D.class, true, 1, methodFilter);
   }
}
