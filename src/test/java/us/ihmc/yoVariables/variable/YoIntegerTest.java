package us.ihmc.yoVariables.variable;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.robotics.Assert;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoIntegerTest
{
   private YoRegistry registry;
   private Random random;
   private YoInteger yoInteger;
   private static final double EPSILON = 1e-10;

   @BeforeEach
   public void setUp()
   {
      registry = new YoRegistry("testRegistry");
      random = new Random(1776L);
      yoInteger = new YoInteger("test", registry);
   }

   @AfterEach
   public void tearDown()
   {
      registry = null;
      yoInteger = null;
   }

   @Test // timeout=300000
   public void testSetAndGet()
   {
      for (int i = 0; i < 100; i++)
      {
         int value = random.nextInt(Integer.MAX_VALUE);
         yoInteger.set(value);
         Assert.assertEquals(value, yoInteger.getIntegerValue());
      }
   }

   @Test // timeout=300000
   public void testIncrementDecrementAddSubtract()
   {
      int value = random.nextInt();
      yoInteger.set(value);

      yoInteger.increment();
      Assert.assertEquals(value + 1, yoInteger.getIntegerValue());

      yoInteger.decrement();
      Assert.assertEquals(value, yoInteger.getIntegerValue());

      yoInteger.add(value);
      Assert.assertEquals(value * 2, yoInteger.getIntegerValue());

      yoInteger.sub(value);
      Assert.assertEquals(value, yoInteger.getIntegerValue());
   }

   @Test // timeout=300000
   public void testLargeValue()
   {
      int value = Integer.MAX_VALUE - 2;
      yoInteger.set(value);
      Assert.assertEquals(value, yoInteger.getIntegerValue());
   }

   @Test // timeout=300000
   public void testValueEquals()
   {
      boolean result = yoInteger.valueEquals(0);
      assertTrue(result);
   }

   @Test // timeout=300000
   public void testSetFinal()
   {
      Assert.assertEquals(0, yoInteger.getIntegerValue());
      yoInteger.set(0);
      Assert.assertEquals(0, yoInteger.getIntegerValue());

      int value = random.nextInt() + 1;
      yoInteger.set(value);
      Assert.assertEquals(value, yoInteger.getIntegerValue());
   }

   @Test // timeout=300000
   public void testSetValueFromDouble()
   {
      double doubleValue = random.nextDouble();
      int intValue = (int) Math.round(doubleValue);
      boolean notifyListeners = true;
      yoInteger.setValueFromDouble(doubleValue, notifyListeners);
      Assert.assertEquals(intValue, yoInteger.getIntegerValue());
   }

   @Test // timeout=300000
   public void testGetValueAsDouble()
   {
      int value = 15;
      Assert.assertEquals(0, yoInteger.getIntegerValue());
      yoInteger.set(value);
      double result = yoInteger.getValueAsDouble();
      assertEquals(15.0, result, EPSILON);
   }

   @Test // timeout=300000
   public void testToString()
   {
      Assert.assertEquals(yoInteger.getName() + ": " + yoInteger.getIntegerValue(), yoInteger.toString());
   }

   @Test // timeout=300000
   public void testGetYoVariableType()
   {
      Assert.assertEquals(YoVariableType.INTEGER, yoInteger.getType());
   }

   @Test // timeout=300000
   public void testGetAndSetValueAsLongBits()
   {
      int value = 57;
      yoInteger.set(value);
      long result = yoInteger.getValueAsLongBits();
      assertEquals(value, result);

      long longValue = 12345;
      boolean notifyListeners = true;
      yoInteger.setValueFromLongBits(longValue, notifyListeners);
      Assert.assertEquals(longValue, yoInteger.getValueAsLongBits());
   }

   @Test // timeout=300000
   public void testDuplicate()
   {
      YoInteger yoInteger2 = new YoInteger("var2", "descriptionTest", registry);
      YoRegistry newRegistry = new YoRegistry("newRegistry");
      YoInteger duplicate = yoInteger2.duplicate(newRegistry);
      Assert.assertEquals(yoInteger2.getName(), duplicate.getName());
      Assert.assertEquals(yoInteger2.getDescription(), duplicate.getDescription());
      Assert.assertEquals(yoInteger2.getLowerBound(), duplicate.getLowerBound(), EPSILON);
      Assert.assertEquals(yoInteger2.getUpperBound(), duplicate.getUpperBound(), EPSILON);
   }

   @Test // timeout = 300000
   public void testProviderValue()
   {
      yoInteger.set(1250948);
      assertEquals(yoInteger.getIntegerValue(), yoInteger.getValue());
      yoInteger.set(-521);
      assertEquals(yoInteger.getIntegerValue(), yoInteger.getValue());

   }
}
