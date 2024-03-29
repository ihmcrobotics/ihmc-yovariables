package us.ihmc.yoVariables.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameQuaternionReadOnlyTest;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.QuaternionBasicsTest;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.YoMutableFrameObject;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoVariable;

public final class YoMutableFrameQuaternionTest extends FrameQuaternionReadOnlyTest<YoMutableFrameQuaternion>
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final double EPSILON = 1e-10;

   @Override
   public YoMutableFrameQuaternion createFrameTuple(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
   {
      if (tuple4DReadOnly instanceof QuaternionReadOnly)
         return new YoMutableFrameQuaternion("", "", null, referenceFrame, (QuaternionReadOnly) tuple4DReadOnly);
      else
         return new YoMutableFrameQuaternion("", "", null, referenceFrame, tuple4DReadOnly);
   }

   @Override
   public YoMutableFrameQuaternion createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setUnsafe(x, y, z, s);
      return new YoMutableFrameQuaternion("", "", null, referenceFrame, quaternion);
   }

   @Test
   public void testMutableFrameObject()
   {
      YoRegistry registry = new YoRegistry("TestRegistry");
      Random random = new Random(4290L);

      List<ReferenceFrame> frames = new ArrayList<>();
      frames.addAll(Arrays.asList(EuclidFrameRandomTools.nextReferenceFrameTree(random)));
      frames.add(null);

      YoMutableFrameObject mutableFrameObject = new YoMutableFrameQuaternion("", "", registry);
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

      { // Test YoMutableFrameQuaternion()
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null);
         assertTrue(YoMutableFrameQuaternion.getReferenceFrame() == worldFrame);
         EuclidCoreTestTools.assertQuaternionIsSetToZero(YoMutableFrameQuaternion);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null, randomFrame);
         assertTrue(YoMutableFrameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertQuaternionIsSetToZero(YoMutableFrameQuaternion);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(ReferenceFrame referenceFrame, double x, double y, double z, double s)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Quaternion randomQuaternion = EuclidCoreRandomTools.nextQuaternion(random);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("",
                                                                                          "",
                                                                                          null,
                                                                                          randomFrame,
                                                                                          randomQuaternion.getX(),
                                                                                          randomQuaternion.getY(),
                                                                                          randomQuaternion.getZ(),
                                                                                          randomQuaternion.getS());
         assertTrue(YoMutableFrameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomQuaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(ReferenceFrame referenceFrame, double[] quaternionArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Quaternion randomQuaternion = EuclidCoreRandomTools.nextQuaternion(random);
         double[] array = new double[4];
         randomQuaternion.get(array);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null, randomFrame, array);
         assertTrue(YoMutableFrameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomQuaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(ReferenceFrame referenceFrame, DMatrixRMaj matrix)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Quaternion randomQuaternion = EuclidCoreRandomTools.nextQuaternion(random);
         DMatrixRMaj denseMatrix = new DMatrixRMaj(4, 1);
         randomQuaternion.get(denseMatrix);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null, randomFrame, denseMatrix);
         assertTrue(YoMutableFrameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomQuaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(ReferenceFrame referenceFrame, QuaternionReadOnly quaternionReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         QuaternionReadOnly randomQuaternion = EuclidCoreRandomTools.nextQuaternion(random);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null, randomFrame, randomQuaternion);
         assertTrue(YoMutableFrameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomQuaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Tuple4DReadOnly randomTuple = EuclidCoreRandomTools.nextVector4D(random);
         Quaternion expectedQuaternion = new Quaternion(randomTuple);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null, randomFrame, randomTuple);
         EuclidCoreTestTools.assertEquals(expectedQuaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         RotationMatrixReadOnly randomRotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Quaternion expectedQuaternion = new Quaternion(randomRotationMatrix);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null, randomFrame, randomRotationMatrix);
         EuclidCoreTestTools.assertEquals(expectedQuaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(ReferenceFrame referenceFrame, AxisAngleReadOnly axisAngle)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         AxisAngleReadOnly randomAxisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         Quaternion expectedQuaternion = new Quaternion(randomAxisAngle);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null, randomFrame, randomAxisAngle);
         EuclidCoreTestTools.assertEquals(expectedQuaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3DReadOnly randomRotationVector = EuclidCoreRandomTools.nextVector3D(random);
         Quaternion expectedQuaternion = new Quaternion(randomRotationVector);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null, randomFrame, randomRotationVector);
         EuclidCoreTestTools.assertEquals(expectedQuaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double pitch = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         Quaternion expectedQuaternion = new Quaternion(yaw, pitch, roll);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null, randomFrame, yaw, pitch, roll);
         EuclidCoreTestTools.assertEquals(expectedQuaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(FrameTuple4DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameTuple4DReadOnly randomFrameTuple4D = EuclidFrameRandomTools.nextFrameQuaternion(random, randomFrame);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null, randomFrameTuple4D);
         assertTrue(YoMutableFrameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomFrameTuple4D, YoMutableFrameQuaternion, EPSILON);
         EuclidFrameTestTools.assertEquals(randomFrameTuple4D, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test YoMutableFrameQuaternion(FrameQuaternionReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameQuaternionReadOnly randomFrameQuaternion = EuclidFrameRandomTools.nextFrameQuaternion(random, randomFrame);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = new YoMutableFrameQuaternion("", "", null, randomFrameQuaternion);
         assertTrue(YoMutableFrameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomFrameQuaternion, YoMutableFrameQuaternion, EPSILON);
         EuclidFrameTestTools.assertEquals(randomFrameQuaternion, YoMutableFrameQuaternion, EPSILON);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         Quaternion expectedGeometryObject = EuclidCoreRandomTools.nextQuaternion(random);
         expectedGeometryObject.setToZero();

         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         YoMutableFrameQuaternion frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(expectedGeometryObject.epsilonEquals(frameGeometryObject, EPSILON));
         frameGeometryObject.setToZero();
         EuclidCoreTestTools.assertEquals(expectedGeometryObject, frameGeometryObject, EPSILON);

         frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         ReferenceFrame newFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(expectedGeometryObject.epsilonEquals(frameGeometryObject, EPSILON));
         frameGeometryObject.setToZero(newFrame);
         assertEquals(newFrame, frameGeometryObject.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(expectedGeometryObject, frameGeometryObject, EPSILON);
      }
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(574);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         YoMutableFrameQuaternion frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(frameGeometryObject.containsNaN());
         frameGeometryObject.setToNaN();
         EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(frameGeometryObject);

         frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         ReferenceFrame newFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(frameGeometryObject.containsNaN());
         frameGeometryObject.setToNaN(newFrame);
         assertEquals(newFrame, frameGeometryObject.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(frameGeometryObject);
      }
   }

   @Test
   public void testMatchingFrame() throws Exception
   {
      Random random = new Random(3225);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameQuaternionReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         YoMutableFrameQuaternion expected = createFrameTuple(sourceFrame, EuclidCoreRandomTools.nextQuaternion(random));
         YoMutableFrameQuaternion actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(expected);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple4DReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         FrameTuple4DBasics source = EuclidFrameRandomTools.nextFrameQuaternion(random, sourceFrame);
         YoMutableFrameQuaternion actual = createFrameTuple(destinationFrame, EuclidCoreRandomTools.nextQuaternion(random));

         actual.setMatchingFrame(source);
         YoMutableFrameQuaternion expected = new YoMutableFrameQuaternion("", "", null, source);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSetIncludingFrame() throws Exception
   {
      Random random = new Random(2342);

      ReferenceFrame initialFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, AxisAngleReadOnly axisAngle)
         AxisAngleReadOnly axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, YoMutableFrameQuaternion.getReferenceFrame());
         YoMutableFrameQuaternion.setIncludingFrame(newFrame, axisAngle);
         quaternion.set(axisAngle);
         assertEquals(newFrame, YoMutableFrameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(quaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix)
         RotationMatrixReadOnly rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, YoMutableFrameQuaternion.getReferenceFrame());
         YoMutableFrameQuaternion.setIncludingFrame(newFrame, rotationMatrix);
         quaternion.set(rotationMatrix);
         assertEquals(newFrame, YoMutableFrameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(quaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
         Vector3DReadOnly rotationVector = EuclidCoreRandomTools.nextRotationVector(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, YoMutableFrameQuaternion.getReferenceFrame());
         YoMutableFrameQuaternion.setRotationVectorIncludingFrame(newFrame, rotationVector);
         quaternion.setRotationVector(rotationVector);
         assertEquals(newFrame, YoMutableFrameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(quaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(FrameVector3DReadOnly rotationVector)
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3DReadOnly rotationVector = new FrameVector3D(newFrame, EuclidCoreRandomTools.nextRotationVector(random));
         YoMutableFrameQuaternion YoMutableFrameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, YoMutableFrameQuaternion.getReferenceFrame());
         YoMutableFrameQuaternion.setRotationVectorIncludingFrame(rotationVector);
         quaternion.setRotationVector(rotationVector);
         assertEquals(newFrame, YoMutableFrameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(quaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setYawPitchRollIncludingFrame(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double pitch = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, YoMutableFrameQuaternion.getReferenceFrame());
         YoMutableFrameQuaternion.setYawPitchRollIncludingFrame(newFrame, yaw, pitch, roll);
         quaternion.setYawPitchRoll(yaw, pitch, roll);
         assertEquals(newFrame, YoMutableFrameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(quaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setEulerIncludingFrame(ReferenceFrame referenceFrame, Vector3DReadOnly eulerAngles)
         Vector3D eulerAngles = EuclidCoreRandomTools.nextRotationVector(random);
         eulerAngles.setY(EuclidCoreTools.clamp(eulerAngles.getY(), YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE));
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, YoMutableFrameQuaternion.getReferenceFrame());
         YoMutableFrameQuaternion.setEulerIncludingFrame(newFrame, eulerAngles);
         quaternion.setEuler(eulerAngles);
         assertEquals(newFrame, YoMutableFrameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(quaternion, YoMutableFrameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setEulerIncludingFrame(ReferenceFrame referenceFrame, double rotX, double rotY, double rotZ)
         double rotX = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double rotY = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double rotZ = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YoMutableFrameQuaternion YoMutableFrameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, YoMutableFrameQuaternion.getReferenceFrame());
         YoMutableFrameQuaternion.setEulerIncludingFrame(newFrame, rotX, rotY, rotZ);
         quaternion.setEuler(rotX, rotY, rotZ);
         assertEquals(newFrame, YoMutableFrameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(quaternion, YoMutableFrameQuaternion, EPSILON);
      }
   }

   @Test
   public void testConsistencyWithQuaternion()
   {
      FrameTypeCopier frameTypeBuilder = (frame, quaternion) -> createFrameTuple(frame, (QuaternionReadOnly) quaternion);
      RandomFramelessTypeBuilder framelessTypeBuilder = EuclidCoreRandomTools::nextQuaternion;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("toString");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                  framelessTypeBuilder,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      RandomFramelessTypeBuilder frameless2DTypeBuilder = (random) -> new Quaternion(createRandom2DFrameTuple(random, ReferenceFrame.getWorldFrame()));
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                  frameless2DTypeBuilder,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", Quaternion.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Quaternion.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Quaternion.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(YoMutableFrameQuaternion.class, Quaternion.class, true, 1, methodFilter);
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

         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         YoMutableFrameQuaternion quaternion = new YoMutableFrameQuaternion("", "", null, initialFrame, expected);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expected.applyTransform(transform);

         quaternion.changeFrame(anotherFrame);
         assertTrue(anotherFrame == quaternion.getReferenceFrame());
         EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(expected, quaternion, EPSILON);

         ReferenceFrame differentRootFrame = ReferenceFrameTools.constructARootFrame("anotherRootFrame");
         try
         {
            quaternion.changeFrame(differentRootFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
      }
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(3452);

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests set(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         QuaternionBasics expected = EuclidCoreRandomTools.nextQuaternion(random);

         int initialFrameIndex = random.nextInt(referenceFrames.length);
         ReferenceFrame initialFrame = referenceFrames[initialFrameIndex];
         YoMutableFrameQuaternion actual = createRandomFrameTuple(random, initialFrame);

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         actual.set(initialFrame, expected);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
         assertEquals(initialFrame, actual.getReferenceFrame());

         actual.set(EuclidCoreRandomTools.nextQuaternion(random));

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         expected.set(actual);

         int differenceFrameIndex = initialFrameIndex + random.nextInt(referenceFrames.length - 1) + 1;
         differenceFrameIndex %= referenceFrames.length;
         ReferenceFrame differentFrame = referenceFrames[differenceFrameIndex];

         try
         {
            actual.set(differentFrame, EuclidCoreRandomTools.nextQuaternion(random));
            fail("Should have thrown a ReferenceFrameMismatchException");
         }
         catch (ReferenceFrameMismatchException e)
         {
            // good
            EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
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

         YoMutableFrameQuaternion expected = createEmptyFrameTuple(anotherFrame);
         expected.changeFrame(initialFrame);

         YoMutableFrameQuaternion actual = createRandomFrameTuple(random, initialFrame);
         actual.setFromReferenceFrame(anotherFrame);
         assertTrue(initialFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoMutableFrameQuaternion frameQuaternion1 = createFrameTuple(worldFrame, EuclidCoreRandomTools.nextQuaternion(random));
         YoMutableFrameQuaternion frameQuaternion2 = new YoMutableFrameQuaternion("", "", null, worldFrame);
         double epsilon = random.nextDouble();

         AxisAngle axisAngleDiff;
         Quaternion difference;

         axisAngleDiff = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), 0.99 * epsilon);
         difference = new Quaternion(axisAngleDiff);
         frameQuaternion2.multiply(frameQuaternion1, difference);
         assertTrue(frameQuaternion1.geometricallyEquals(frameQuaternion2, epsilon));

         axisAngleDiff = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), 1.01 * epsilon);
         difference = new Quaternion(axisAngleDiff);
         frameQuaternion2.multiply(frameQuaternion1, difference);
         assertFalse(frameQuaternion1.geometricallyEquals(frameQuaternion2, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(763);

      for (int i = 0; i < ITERATIONS; i++)
      {
         FrameQuaternion expected = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         YoMutableFrameQuaternion actual = new YoMutableFrameQuaternion("", "", null, worldFrame, expected);

         assertEquals(expected.hashCode(), actual.hashCode());
      }
   }

   @Test
   public void testQuaternionBasicsFeatures() throws Exception
   {
      QuaternionBasicsTest<YoMutableFrameQuaternion> quaternionBasicsTest = new QuaternionBasicsTest<YoMutableFrameQuaternion>()
      {
         @Override
         public YoMutableFrameQuaternion createEmptyTuple()
         {
            return new YoMutableFrameQuaternion("", "", null);
         }

         @Override
         public YoMutableFrameQuaternion createTuple(double v, double v1, double v2, double v3)
         {
            YoMutableFrameQuaternion ret = new YoMutableFrameQuaternion("", "", null, ReferenceFrame.getWorldFrame());
            ret.setUnsafe(v, v1, v2, v3);
            return ret;
         }

         @Override
         public YoMutableFrameQuaternion createRandomTuple(Random random)
         {
            return new YoMutableFrameQuaternion("", "", null, ReferenceFrame.getWorldFrame(), EuclidCoreRandomTools.nextQuaternion(random));
         }

         @Override
         public double getEpsilon()
         {
            return 1e-10;
         }
      };

      for (Method testMethod : quaternionBasicsTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         testMethod.invoke(quaternionBasicsTest);
      }
   }
}
