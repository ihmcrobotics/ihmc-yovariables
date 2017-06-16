package us.ihmc.yoVariables.variable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;


public class YoBooleanTest
{
   private YoBoolean yoBoolean;
   private YoVariableRegistry registry;
   private static final double EPSILON = 1e-10;

   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      yoBoolean = new YoBoolean("booleanVariable", registry);
   }

   @After
   public void tearDown()
   {
      yoBoolean = null;
      registry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testBooleanYoVariable()
   {
      Assert.assertEquals("booleanVariable", yoBoolean.getName());
      Assert.assertEquals("testRegistry", yoBoolean.getYoVariableRegistry().getName());
      Assert.assertEquals(false, yoBoolean.getBooleanValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testValueEquals()
   {
      yoBoolean.set(false);
      assertFalse(yoBoolean.valueEquals(true));

      yoBoolean.set(true);
      assertTrue(yoBoolean.valueEquals(true));

      yoBoolean.set(false);
      assertTrue(yoBoolean.valueEquals(false));

      yoBoolean.set(true);
      assertFalse(yoBoolean.valueEquals(false));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetBooleanYoVariable()
   {
      yoBoolean.set(false);
      assertFalse(yoBoolean.getBooleanValue());

      yoBoolean.set(true);
      assertTrue(yoBoolean.getBooleanValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSet_boolean_boolean()
   {
      assertFalse(yoBoolean.getBooleanValue());

      boolean value = true;
      boolean notifyListeners = true;
      boolean result = yoBoolean.set(value, notifyListeners);
      assertTrue(yoBoolean.getBooleanValue());
      assertTrue(result);

      result = yoBoolean.set(value, notifyListeners);
      assertFalse(result);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetAsDouble()
   {
      Random rng = new Random();
      double testRandom = 0;

      yoBoolean.setValueFromDouble(0.0);
      assertFalse(yoBoolean.getBooleanValue());

      testRandom = (double) rng.nextInt(23000) / (double) rng.nextInt(5);

      yoBoolean.setValueFromDouble(10000);
      assertTrue(yoBoolean.getBooleanValue());

      yoBoolean.setValueFromDouble(-0.4);
      assertFalse(yoBoolean.getBooleanValue());
      
      for(int counter = 0; counter < 100; counter++)
      {
	      testRandom = rng.nextDouble();
	      yoBoolean.setValueFromDouble(testRandom);
	      if (testRandom >= 0.5)
	         assertTrue(yoBoolean.getBooleanValue());
	      else
	         assertFalse(yoBoolean.getBooleanValue());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueAsDouble()
   {
      assertFalse(yoBoolean.getBooleanValue());
      double result = yoBoolean.getValueAsDouble();
      assertEquals(0.0, result, EPSILON);

      yoBoolean.set(true);
      assertTrue(yoBoolean.getBooleanValue());
      result = yoBoolean.getValueAsDouble();
      assertEquals(1.0, result, EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testToString()
   {
      yoBoolean.set(false);
      Assert.assertEquals(yoBoolean.toString(), "booleanVariable: false");
      yoBoolean.set(true);
      Assert.assertEquals(yoBoolean.toString(), "booleanVariable: true");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueWithStringBuffer()
   {
      StringBuffer testStringBuffer = new StringBuffer();
      testStringBuffer.append("Test Case: ");
      yoBoolean.set(false);
      yoBoolean.getValueString(testStringBuffer);
      testStringBuffer.append(" ");
      yoBoolean.getValueStringFromDouble(testStringBuffer, 1.0);

      StringBuffer expectedTestStringBuffer = new StringBuffer();
      expectedTestStringBuffer.append("Test Case: false true");

      assertEquals(testStringBuffer.toString(), expectedTestStringBuffer.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetValueAsLongBits()
   {
      assertFalse(yoBoolean.getBooleanValue());
      long value = yoBoolean.getValueAsLongBits();
      assertEquals(0, value);

      yoBoolean.set(true);
      assertTrue(yoBoolean.getBooleanValue());
      value = yoBoolean.getValueAsLongBits();
      assertEquals(1, value);

      boolean notifyListeners = true;
      yoBoolean.setValueFromLongBits(value, notifyListeners);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testYoVariableType()
   {
      Assert.assertEquals(yoBoolean.getYoVariableType(), YoVariableType.BOOLEAN);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDuplicate()
   {
	   YoVariableRegistry newRegistry = new YoVariableRegistry("newTestRegistry");
	   YoBoolean val, testVal;
	   
	   yoBoolean.set(true);
	   val = yoBoolean.duplicate(newRegistry);
	   testVal = (YoBoolean)newRegistry.getAllVariables().get(0);
	   Assert.assertEquals(yoBoolean.getBooleanValue(), val.getBooleanValue());
	   Assert.assertEquals(yoBoolean.getBooleanValue(), testVal.getBooleanValue());
	   assertTrue(val.getYoVariableRegistry().areEqual(newRegistry));
	   assertTrue(val.getYoVariableRegistry().areEqual(testVal.getYoVariableRegistry()));
	   newRegistry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValue()
   {
	   boolean valSet = false;
	   YoVariableRegistry newRegistry = new YoVariableRegistry("newTestRegistry");
	   YoBoolean testBoolean = new YoBoolean("testBooleanVariable", newRegistry);
	   testBoolean.set(true);
	   valSet = yoBoolean.setValue(testBoolean, false);
	   assertTrue(valSet);
	   Assert.assertEquals(yoBoolean.getBooleanValue(), testBoolean.getBooleanValue());
	   assertTrue(yoBoolean.getYoVariableRegistry().areEqual(registry));
   }
   
}