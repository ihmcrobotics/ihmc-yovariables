package us.ihmc.yoVariables.variable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.text.DecimalFormat;
import java.text.FieldPosition;
import java.text.NumberFormat;
import java.util.Random;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoDoubleTest
{

   private YoVariableRegistry registry;
   private YoDouble yoDouble1;
   private YoDouble yoDouble2;
   private static final double EPSILON = 1e-10;
   private Random random = new Random(345345L);

   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      yoDouble1 = new YoDouble("yoDouble1", registry);
      yoDouble2 = new YoDouble("yoDouble2", "description2", registry, 0.0, 10.0);
   }

   @After
   public void tearDown()
   {
      yoDouble1 = null;
      registry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDoubleYoVariableConstructorWithoutDescription()
   {
      assertTrue(yoDouble1.getDoubleValue() == 0.0);
      Assert.assertEquals(yoDouble1.getName(), "yoDouble1");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDoubleYoVariableConstructorWithDescription()
   {
      String testDescription = "This is a test description.";
      YoDouble yoDoubleWithDescription = new YoDouble("yoDoubleWithDescription", testDescription, registry);
      assertTrue(yoDoubleWithDescription.getDoubleValue() == 0.0);
      Assert.assertEquals(yoDoubleWithDescription.getName(), "yoDoubleWithDescription");
      assertTrue(yoDoubleWithDescription.getDescription() == testDescription);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testToString()
   {
      double randomNumber = Math.random();
      yoDouble1.set(randomNumber);
      Assert.assertEquals(yoDouble1.toString(), "yoDouble1: " + randomNumber);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIsNaN()
   {
      assertFalse(yoDouble2.isNaN());
      yoDouble2.set(Double.NaN);
      assertTrue(yoDouble2.isNaN());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAdditionWithDoubles()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      yoDouble1.set(randomNumber1);
      yoDouble1.add(randomNumber2);
      double expectedSum = randomNumber1 + randomNumber2;
      assertTrue(yoDouble1.getDoubleValue() == expectedSum);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSubtractionWithDoubles()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      yoDouble1.set(randomNumber1);
      yoDouble1.sub(randomNumber2);
      double expectedDifference = randomNumber1 - randomNumber2;
      assertTrue(yoDouble1.getDoubleValue() == expectedDifference);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMultiplicationWithDoubles()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      yoDouble1.set(randomNumber1);
      yoDouble1.mul(randomNumber2);
      double expectedProduct = randomNumber1 * randomNumber2;
      assertTrue(yoDouble1.getDoubleValue() == expectedProduct);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAdditionWithDoubleYoVariables()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      yoDouble1.set(randomNumber1);
      yoDouble2.set(randomNumber2);
      double expectedSum = yoDouble1.getDoubleValue() + yoDouble2.getDoubleValue();
      yoDouble1.add(yoDouble2);
      assertTrue(yoDouble1.getDoubleValue() == expectedSum);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSubtractionWithDoubleYoVariables()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      yoDouble1.set(randomNumber1);
      yoDouble2.set(randomNumber2);
      double expectedDifference = yoDouble1.getDoubleValue() - yoDouble2.getDoubleValue();
      yoDouble1.sub(yoDouble2);
      assertTrue(yoDouble1.getDoubleValue() == expectedDifference);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMultiplicationWithDoubleYoVariables()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      yoDouble1.set(randomNumber1);
      yoDouble2.set(randomNumber2);
      double expectedProduct = yoDouble1.getDoubleValue() * yoDouble2.getDoubleValue();
      yoDouble1.mul(yoDouble2);
      assertTrue(yoDouble1.getDoubleValue() == expectedProduct);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testValueEquals()
   {
      double randomNumber = Math.random();
      yoDouble1.set(randomNumber);
      yoDouble1.valueEquals(randomNumber);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValueWithStringBuffer()
   {
      FieldPosition fieldPosition = new FieldPosition(NumberFormat.INTEGER_FIELD);
      NumberFormat doubleFormat = new DecimalFormat(" 0.00000;-0.00000");

      double randomNumber = 100000 * Math.random();
      randomNumber = Math.round(randomNumber);
      randomNumber /= 100000; //five decimal format as determined by doubleFormat under <p>getValue</p> method.
      StringBuffer stringBufferTest = new StringBuffer();
      stringBufferTest.append("Test case:");

      yoDouble1.set(randomNumber);
      yoDouble1.getValueString(stringBufferTest);

      StringBuffer expectedStringBuffer = new StringBuffer();
      expectedStringBuffer.append("Test case:");
      doubleFormat.format(randomNumber, expectedStringBuffer, fieldPosition);
      //         expectedStringBuffer.append(" " + randomNumber); //added space that occurs in <p>getValue</p> method.

      System.out.println("Expected String Buffer: " + expectedStringBuffer.toString());
      System.out.println("String Buffer: " + stringBufferTest.toString());

      assertEquals(expectedStringBuffer.toString(), stringBufferTest.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetMethods()
   {
      double randomNumber = Math.random();
      yoDouble1.set(randomNumber);
      yoDouble2.set(yoDouble1.getDoubleValue());
      assertTrue(yoDouble1.getDoubleValue() == yoDouble2.getDoubleValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetDoubleValue()
   {
      double randomNumber = Math.random();
      yoDouble1.setValueFromDouble(randomNumber);
      yoDouble2.setValueFromDouble(yoDouble1.getValueAsDouble());
      assertTrue(yoDouble1.getValueAsDouble() == yoDouble2.getValueAsDouble());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetFinal()
   {
      yoDouble1.set(0.0);
      yoDouble1.set(0.0);
      
      yoDouble1.set(10.0);
      Assert.assertEquals(10.0, yoDouble1.getDoubleValue(), EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueAsLongBitsAndSetValueFromLongBits()
   {
      long longValue = random.nextLong();
      yoDouble1.setValueFromLongBits(longValue);
      Assert.assertEquals(longValue, yoDouble1.getValueAsLongBits());
      
      YoDouble yoDouble = new YoDouble("doubleYo", registry);
      yoDouble.set(Double.NEGATIVE_INFINITY);
      
      yoDouble1.setValue(yoDouble, true);
      Assert.assertEquals(0xfff0000000000000L, yoDouble1.getValueAsLongBits());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetYoVariableType()
   {
      assertTrue(yoDouble1.getYoVariableType() == YoVariableType.DOUBLE);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDuplicate()
   {
      String newName = "registry2000";
      double value = random.nextDouble();
      yoDouble2.set(value);
      YoVariableRegistry newRegistry = new YoVariableRegistry(newName);
      YoDouble duplicate = yoDouble2.duplicate(newRegistry);
      Assert.assertEquals(yoDouble2.getDoubleValue(), duplicate.getDoubleValue(), EPSILON);
   }
	
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProviderValue()
   {
      yoDouble1.set(12509481.0);
      yoDouble2.set(2358);
      
      assertEquals(yoDouble1.getDoubleValue(), yoDouble1.getValue(), 1e-9);
      assertEquals(yoDouble2.getDoubleValue(), yoDouble2.getValue(), 1e-9);
   }
}