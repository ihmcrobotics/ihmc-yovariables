package us.ihmc.yoVariables.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameTuple3DBasicsTest;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
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
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.YoMutableFrameObject;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoMutableFrameVector3DTest extends FrameTuple3DBasicsTest<YoMutableFrameVector3D>
{
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
   public void testMutableFrameObject()
   {
      YoRegistry registry = new YoRegistry("TestRegistry");
      Random random = new Random(4290L);

      List<ReferenceFrame> frames = new ArrayList<>();
      frames.addAll(Arrays.asList(EuclidFrameRandomTools.nextReferenceFrameTree(random)));
      frames.add(null);

      YoMutableFrameObject mutableFrameObject = new YoMutableFrameVector3D("", "", registry);
      YoVariable frameIndex = registry.findVariable(YoGeometryNameTools.assembleName("", "frame", ""));
      assertNotNull(frameIndex);

      for (int i = 0; i < 1000; i++)
      {
         ReferenceFrame frame = frames.get(random.nextInt(frames.size()));
         mutableFrameObject.setReferenceFrame(frame);
         assertTrue(frame == mutableFrameObject.getReferenceFrame());
         if (frame == null)
         {
            assertEquals(FrameIndexMap.NO_ENTRY_KEY, frameIndex.getValueAsLongBits());
         }
         else
         {
            assertEquals(frame.getFrameIndex(), frameIndex.getValueAsLongBits());
         }
      }
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
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("",
                                                                                    "",
                                                                                    null,
                                                                                    randomFrame,
                                                                                    randomTuple.getX(),
                                                                                    randomTuple.getY(),
                                                                                    randomTuple.getZ());
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, YoMutableFrameVector3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.nextVector3D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomFrame, array);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, YoMutableFrameVector3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomTuple = EuclidCoreRandomTools.nextVector3D(random);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomFrame, randomTuple);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, YoMutableFrameVector3D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector2D randomTuple2D = EuclidCoreRandomTools.nextVector2D(random);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomFrame, randomTuple2D);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple2D, new Vector2D(YoMutableFrameVector3D), EPSILON);
         assertTrue(YoMutableFrameVector3D.getZ() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector2D randomFrameTuple2D = EuclidFrameRandomTools.nextFrameVector2D(random, randomFrame);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomFrameTuple2D);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomFrameTuple2D, new Vector2D(YoMutableFrameVector3D), EPSILON);
         assertTrue(YoMutableFrameVector3D.getZ() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameVector3D(FrameTuple3DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameTuple3DReadOnly randomTuple = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         YoMutableFrameVector3D YoMutableFrameVector3D = new YoMutableFrameVector3D("", "", null, randomTuple);
         assertTrue(YoMutableFrameVector3D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, YoMutableFrameVector3D, EPSILON);
         EuclidFrameTestTools.assertEquals(randomTuple, YoMutableFrameVector3D, EPSILON);
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

         EuclidFrameTestTools.assertEquals(expected, actual, EPSILON);
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

         EuclidFrameTestTools.assertEquals(expected, actual, EPSILON);
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
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);

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
         FrameVector3D expected = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame, -1.0e15, 1.0e15);
         YoMutableFrameVector3D actual = new YoMutableFrameVector3D("", "", null, worldFrame, expected);

         assertEquals(expected.hashCode(), actual.hashCode());
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", Vector3D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Vector3D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Vector3D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(YoMutableFrameVector3D.class, Vector3D.class, true, 1, methodFilter);
   }
}
