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

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple2DBasicsTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoMutableFramePoint2DTest extends FrameTuple2DBasicsTest<YoMutableFramePoint2D>
{
   static
   {
      YoVariable.SAVE_STACK_TRACE = false;
      YoVariable.warnAboutNullRegistries = false;
   }

   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public Tuple2DBasics createRandomFramelessTuple(Random random)
   {
      return EuclidCoreRandomTools.nextPoint2D(random);
   }

   @Override
   public YoMutableFramePoint2D createFrameTuple(ReferenceFrame referenceFrame, double x, double y)
   {
      return new YoMutableFramePoint2D("", "", null, referenceFrame, x, y);
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test YoMutableFramePoint2D()
         YoMutableFramePoint2D YoMutableFramePoint2D = new YoMutableFramePoint2D("", "", null);
         assertTrue(YoMutableFramePoint2D.getReferenceFrame() == worldFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(YoMutableFramePoint2D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint2D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YoMutableFramePoint2D YoMutableFramePoint2D = new YoMutableFramePoint2D("", "", null, randomFrame);
         assertTrue(YoMutableFramePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(YoMutableFramePoint2D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint2D(ReferenceFrame referenceFrame, double x, double y)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point2D randomTuple = EuclidCoreRandomTools.nextPoint2D(random);
         YoMutableFramePoint2D YoMutableFramePoint2D = new YoMutableFramePoint2D("", "", null, randomFrame, randomTuple.getX(), randomTuple.getY());
         assertTrue(YoMutableFramePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, YoMutableFramePoint2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint2D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point2D randomTuple = EuclidCoreRandomTools.nextPoint2D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         YoMutableFramePoint2D YoMutableFramePoint2D = new YoMutableFramePoint2D("", "", null, randomFrame, array);
         assertTrue(YoMutableFramePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, YoMutableFramePoint2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint2D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point2D randomTuple = EuclidCoreRandomTools.nextPoint2D(random);
         YoMutableFramePoint2D YoMutableFramePoint2D = new YoMutableFramePoint2D("", "", null, randomFrame, randomTuple);
         assertTrue(YoMutableFramePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, YoMutableFramePoint2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint2D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point3D randomTuple3D = EuclidCoreRandomTools.nextPoint3D(random);
         YoMutableFramePoint2D YoMutableFramePoint2D = new YoMutableFramePoint2D("", "", null, randomFrame, randomTuple3D);
         assertTrue(YoMutableFramePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(new Point2D(randomTuple3D), YoMutableFramePoint2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint3D(FrameTuple3DReadOnly frameTuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint3D randomFrameTuple3D = EuclidFrameRandomTools.nextFramePoint3D(random, randomFrame);
         YoMutableFramePoint2D YoMutableFramePoint2D = new YoMutableFramePoint2D("", "", null, randomFrameTuple3D);
         assertTrue(YoMutableFramePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(new Point2D(randomFrameTuple3D), YoMutableFramePoint2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFramePoint2D(FrameTuple2DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint2DReadOnly randomTuple = EuclidFrameRandomTools.nextFramePoint2D(random, randomFrame);
         YoMutableFramePoint2D YoMutableFramePoint2D = new YoMutableFramePoint2D("", "", null, randomTuple);
         assertTrue(YoMutableFramePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DEquals(randomTuple, YoMutableFramePoint2D, EPSILON);
         EuclidFrameTestTools.assertFrameTuple2DEquals(randomTuple, YoMutableFramePoint2D, EPSILON);
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
         YoMutableFramePoint2D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source);

         YoMutableFramePoint2D expected = new YoMutableFramePoint2D("", "", null, source);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertFrameTuple2DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      Random random = new Random(4353);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random, true);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         Point2D expected = EuclidCoreRandomTools.nextPoint2D(random);
         YoMutableFramePoint2D actual = new YoMutableFramePoint2D("", "", null, initialFrame, expected);

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

         Point2D expected = EuclidCoreRandomTools.nextPoint2D(random);
         YoMutableFramePoint2D actual = new YoMutableFramePoint2D("", "", null, initialFrame, expected);

         expected.applyTransform(initialFrame.getTransformToDesiredFrame(newFrame), false);
         actual.changeFrameAndProjectToXYPlane(newFrame);

         assertTrue(newFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSetFromReferenceFrame() throws Exception
   {
      Random random = new Random(6572);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random, true);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         YoMutableFramePoint2D expected = createEmptyFrameTuple(anotherFrame);
         expected.changeFrame(initialFrame);

         YoMutableFramePoint2D actual = createRandomFrameTuple(random, initialFrame);
         actual.setFromReferenceFrame(anotherFrame);
         assertTrue(initialFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoMutableFramePoint2D framePoint1 = createRandomFrameTuple(random, worldFrame);
         YoMutableFramePoint2D framePoint2 = new YoMutableFramePoint2D("", "", null, worldFrame);
         double epsilon = random.nextDouble();
         Vector2D difference;

         difference = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.99 * epsilon);
         framePoint2.add(framePoint1, difference);
         assertTrue(framePoint1.geometricallyEquals(framePoint2, epsilon));

         difference = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.01 * epsilon);
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
         Point2D expected = EuclidCoreRandomTools.nextPoint2D(random, -1.0e15, 1.0e15);
         YoMutableFramePoint2D actual = new YoMutableFramePoint2D("", "", null, worldFrame, expected);

         assertEquals(expected.hashCode(), actual.hashCode());
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", Point2D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Point2D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Point2D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(YoMutableFramePoint2D.class, Point2D.class, true, 1, methodFilter);
   }
}
