package us.ihmc.yoVariables.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;

public class YoFrameUnitVector2DTest
{
   @Test
   public void testAPIOverloading()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", UnitVector2DReadOnly.class));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      tester.assertOverloadingWithFrameObjects(YoFrameUnitVector2D.class, UnitVector2DBasics.class, false, 1, methodFilter);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(YoFrameUnitVector2DTest::nextYoFrameUnitVector2D,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithUnitVector2D()
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("epsilonEquals")
            && !m.getName().contains("IntermediateVariableSupplier");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality((referenceFrame, unitVector2D) -> newYoFrameUnitVector2D(referenceFrame,
                                                                                                                           (UnitVector2DReadOnly) unitVector2D),
                                                                  EuclidCoreRandomTools::nextUnitVector2D,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testSetMatchingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(YoFrameUnitVector2DTest::nextYoFrameUnitVector2D,
                                                         EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   private static YoFrameUnitVector2D nextYoFrameUnitVector2D(Random random, ReferenceFrame referenceFrame)
   {
      YoFrameUnitVector2D next = new YoFrameUnitVector2D("", referenceFrame, null);
      next.set(EuclidCoreRandomTools.nextUnitVector2D(random));
      return next;
   }

   private static YoFrameUnitVector2D newYoFrameUnitVector2D(ReferenceFrame referenceFrame, UnitVector2DReadOnly unitVector2D)
   {
      YoFrameUnitVector2D next = new YoFrameUnitVector2D("", referenceFrame, null);
      next.set(unitVector2D);
      return next;
   }
}
