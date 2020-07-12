package us.ihmc.yoVariables.variable;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.robotics.Assert;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoDoubleTest
{

   private YoRegistry registry;
   private YoDouble yoDouble1;
   private YoDouble yoDouble2;
   private static final double EPSILON = 1e-10;
   private Random random = new Random(345345L);

   @BeforeEach
   public void setUp()
   {
      registry = new YoRegistry("testRegistry");
      yoDouble1 = new YoDouble("yoDouble1", registry);
      yoDouble2 = new YoDouble("yoDouble2", "description2", registry, 0.0, 10.0);
   }

   @AfterEach
   public void tearDown()
   {
      yoDouble1 = null;
      registry = null;
   }

   @Test // timeout=300000
   public void testDoubleYoVariableConstructorWithoutDescription()
   {
      assertTrue(yoDouble1.getDoubleValue() == 0.0);
      Assert.assertEquals(yoDouble1.getName(), "yoDouble1");
   }

   @Test // timeout=300000
   public void testDoubleYoVariableConstructorWithDescription()
   {
      String testDescription = "This is a test description.";
      YoDouble yoDoubleWithDescription = new YoDouble("yoDoubleWithDescription", testDescription, registry);
      assertTrue(yoDoubleWithDescription.getDoubleValue() == 0.0);
      Assert.assertEquals(yoDoubleWithDescription.getName(), "yoDoubleWithDescription");
      assertTrue(yoDoubleWithDescription.getDescription() == testDescription);
   }

   @Test // timeout=300000
   public void testToString()
   {
      double randomNumber = Math.random();
      yoDouble1.set(randomNumber);
      Assert.assertEquals(yoDouble1.toString(), "yoDouble1: " + randomNumber);
   }

   @Test // timeout=300000
   public void testIsNaN()
   {
      assertFalse(yoDouble2.isNaN());
      yoDouble2.set(Double.NaN);
      assertTrue(yoDouble2.isNaN());
   }

   @Test // timeout=300000
   public void testAdditionWithDoubles()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      yoDouble1.set(randomNumber1);
      yoDouble1.add(randomNumber2);
      double expectedSum = randomNumber1 + randomNumber2;
      assertTrue(yoDouble1.getDoubleValue() == expectedSum);
   }

   @Test // timeout=300000
   public void testSubtractionWithDoubles()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      yoDouble1.set(randomNumber1);
      yoDouble1.sub(randomNumber2);
      double expectedDifference = randomNumber1 - randomNumber2;
      assertTrue(yoDouble1.getDoubleValue() == expectedDifference);
   }

   @Test // timeout=300000
   public void testMultiplicationWithDoubles()
   {
      double randomNumber1 = Math.random();
      double randomNumber2 = Math.random();

      yoDouble1.set(randomNumber1);
      yoDouble1.mul(randomNumber2);
      double expectedProduct = randomNumber1 * randomNumber2;
      assertTrue(yoDouble1.getDoubleValue() == expectedProduct);
   }

   @Test // timeout=300000
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

   @Test // timeout=300000
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

   @Test // timeout=300000
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

   @Test // timeout=300000
   public void testValueEquals()
   {
      double randomNumber = Math.random();
      yoDouble1.set(randomNumber);
      yoDouble1.valueEquals(randomNumber);
   }

   @Test // timeout=300000
   public void testGetAndSetMethods()
   {
      double randomNumber = Math.random();
      yoDouble1.set(randomNumber);
      yoDouble2.set(yoDouble1.getDoubleValue());
      assertTrue(yoDouble1.getDoubleValue() == yoDouble2.getDoubleValue());
   }

   @Test // timeout=300000
   public void testGetAndSetDoubleValue()
   {
      double randomNumber = Math.random();
      yoDouble1.setValueFromDouble(randomNumber);
      yoDouble2.setValueFromDouble(yoDouble1.getValueAsDouble());
      assertTrue(yoDouble1.getValueAsDouble() == yoDouble2.getValueAsDouble());
   }

   @Test // timeout=300000
   public void testSetFinal()
   {
      yoDouble1.set(0.0);
      yoDouble1.set(0.0);

      yoDouble1.set(10.0);
      Assert.assertEquals(10.0, yoDouble1.getDoubleValue(), EPSILON);
   }

   @Test // timeout=300000
   public void testGetYoVariableType()
   {
      assertTrue(yoDouble1.getType() == YoVariableType.DOUBLE);
   }

   @Test // timeout=300000
   public void testDuplicate()
   {
      String newName = "registry2000";
      double value = random.nextDouble();
      yoDouble2.set(value);
      YoRegistry newRegistry = new YoRegistry(newName);
      YoDouble duplicate = yoDouble2.duplicate(newRegistry);
      Assert.assertEquals(yoDouble2.getDoubleValue(), duplicate.getDoubleValue(), EPSILON);
   }

   @Test // timeout = 300000
   public void testProviderValue()
   {
      yoDouble1.set(12509481.0);
      yoDouble2.set(2358);

      assertEquals(yoDouble1.getDoubleValue(), yoDouble1.getValue(), 1e-9);
      assertEquals(yoDouble2.getDoubleValue(), yoDouble2.getValue(), 1e-9);
   }
}