package us.ihmc.yoVariables.variable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class IntegerYoVariableTest
{
   private YoVariableRegistry registry;
   private Random random;
   private IntegerYoVariable integerYoVariable;
   private static final double EPSILON = 1e-10;
   
   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      random = new Random(1776L);
      integerYoVariable = new IntegerYoVariable("test", registry);
   }
   
   @After
   public void tearDown()
   {
      registry = null;
      integerYoVariable = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetAndGet()
   {
      for (int i = 0; i < 100; i++)
      {
         int value = random.nextInt(Integer.MAX_VALUE);
         integerYoVariable.set(value);
         Assert.assertEquals(value, integerYoVariable.getIntegerValue());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIncrementDecrementAddSubtract()
   {
      int value = random.nextInt();
      integerYoVariable.set(value);
      
      integerYoVariable.increment();
      Assert.assertEquals(value + 1, integerYoVariable.getIntegerValue());
      
      integerYoVariable.decrement();
      Assert.assertEquals(value, integerYoVariable.getIntegerValue());
      
      integerYoVariable.add(value);
      Assert.assertEquals(value * 2, integerYoVariable.getIntegerValue());
      
      integerYoVariable.subtract(value);
      Assert.assertEquals(value, integerYoVariable.getIntegerValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testLargeValue()
   {
      int value = Integer.MAX_VALUE - 2;
      integerYoVariable.set(value);
      Assert.assertEquals(value, integerYoVariable.getIntegerValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testValueEquals()
   {
      boolean result = integerYoVariable.valueEquals(0);
      assertTrue(result);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetFinal()
   {
      Assert.assertEquals(0, integerYoVariable.getIntegerValue());
      integerYoVariable.set(0);
      Assert.assertEquals(0, integerYoVariable.getIntegerValue());
      
      int value = random.nextInt() + 1;
      integerYoVariable.set(value);
      Assert.assertEquals(value, integerYoVariable.getIntegerValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValueFromDouble()
   {
      double doubleValue = random.nextDouble();
      int intValue = (int) Math.round(doubleValue);
      boolean notifyListeners = true;
      integerYoVariable.setValueFromDouble(doubleValue, notifyListeners);
      Assert.assertEquals(intValue, integerYoVariable.getIntegerValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueAsDouble()
   {
      int value = 15;
      Assert.assertEquals(0, integerYoVariable.getIntegerValue());
      integerYoVariable.set(value);
      double result = integerYoVariable.getValueAsDouble();
      assertEquals(15.0, result, EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testToString()
{
   Assert.assertEquals(integerYoVariable.getName() + ": " + integerYoVariable.getIntegerValue(), integerYoVariable.toString());
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testGetValueString()
{
   Assert.assertEquals(0, integerYoVariable.getIntegerValue());
   int value = random.nextInt();
   integerYoVariable.set(value);
   StringBuffer stringBuffer = new StringBuffer();
   integerYoVariable.getValueString(stringBuffer);
   assertEquals("" + value, stringBuffer.toString());
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testGetValueStringFromDouble()
{
   Assert.assertEquals(0, integerYoVariable.getIntegerValue());
   double doubleValue = random.nextDouble();
   int value = (int) Math.round(doubleValue);
   integerYoVariable.setValueFromDouble(doubleValue);
   StringBuffer stringBuffer = new StringBuffer();
   integerYoVariable.getValueStringFromDouble(stringBuffer, doubleValue);
   assertEquals("" + value, stringBuffer.toString()); 
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testGetYoVariableType()
{
   Assert.assertEquals(YoVariableType.INTEGER, integerYoVariable.getYoVariableType());
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testGetAndSetValueAsLongBits()
{
   int value = 57;
   integerYoVariable.set(value);
   long result = integerYoVariable.getValueAsLongBits();
   assertEquals(value, result);
   
   long longValue = 12345;
   boolean notifyListeners = true;
   integerYoVariable.setValueFromLongBits(longValue, notifyListeners);
   Assert.assertEquals(longValue, integerYoVariable.getValueAsLongBits());
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testDuplicate()
{
   IntegerYoVariable integerYoVariable2 = new IntegerYoVariable("var2", "descriptionTest", registry);
   YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry");
   IntegerYoVariable duplicate = integerYoVariable2.duplicate(newRegistry);
   Assert.assertEquals(integerYoVariable2.getName(), duplicate.getName());
   Assert.assertEquals(integerYoVariable2.getDescription(), duplicate.getDescription());
   Assert.assertEquals(integerYoVariable2.getManualScalingMin(), duplicate.getManualScalingMin(), EPSILON);
   Assert.assertEquals(integerYoVariable2.getManualScalingMax(), duplicate.getManualScalingMax(), EPSILON);
}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
public void testSetValue()
{
   IntegerYoVariable integerYoVariable2 = new IntegerYoVariable("var2", "descriptionTest", registry);
   boolean notifyListeners = true;
   integerYoVariable.setValue(integerYoVariable2, notifyListeners);
   Assert.assertEquals(integerYoVariable2.getIntegerValue(), integerYoVariable.getIntegerValue());
}

}
