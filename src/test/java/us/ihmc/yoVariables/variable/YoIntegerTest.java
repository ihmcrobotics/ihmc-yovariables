package us.ihmc.yoVariables.variable;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

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
         assertEquals(value, yoInteger.getIntegerValue());
      }
   }

   @Test // timeout=300000
   public void testIncrementDecrementAddSubtract()
   {
      int value = random.nextInt();
      yoInteger.set(value);

      yoInteger.increment();
      assertEquals(value + 1, yoInteger.getIntegerValue());

      yoInteger.decrement();
      assertEquals(value, yoInteger.getIntegerValue());

      yoInteger.add(value);
      assertEquals(value * 2, yoInteger.getIntegerValue());

      yoInteger.sub(value);
      assertEquals(value, yoInteger.getIntegerValue());
   }

   @Test // timeout=300000
   public void testLargeValue()
   {
      int value = Integer.MAX_VALUE - 2;
      yoInteger.set(value);
      assertEquals(value, yoInteger.getIntegerValue());
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
      assertEquals(0, yoInteger.getIntegerValue());
      yoInteger.set(0);
      assertEquals(0, yoInteger.getIntegerValue());

      int value = random.nextInt() + 1;
      yoInteger.set(value);
      assertEquals(value, yoInteger.getIntegerValue());
   }

   @Test // timeout=300000
   public void testSetValueFromDouble()
   {
      double doubleValue = random.nextDouble();
      int intValue = (int) Math.round(doubleValue);
      boolean notifyListeners = true;
      yoInteger.setValueFromDouble(doubleValue, notifyListeners);
      assertEquals(intValue, yoInteger.getIntegerValue());
   }

   @Test // timeout=300000
   public void testGetValueAsDouble()
   {
      int value = 15;
      assertEquals(0, yoInteger.getIntegerValue());
      yoInteger.set(value);
      double result = yoInteger.getValueAsDouble();
      assertEquals(15.0, result, EPSILON);
   }

   @Test // timeout=300000
   public void testToString()
   {
      assertEquals(yoInteger.getName() + ": " + yoInteger.getIntegerValue(), yoInteger.toString());
   }

   @Test // timeout=300000
   public void testGetYoVariableType()
   {
      assertEquals(YoVariableType.INTEGER, yoInteger.getType());
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
      assertEquals(longValue, yoInteger.getValueAsLongBits());
   }

   @Test // timeout=300000
   public void testDuplicate()
   {
      YoInteger yoInteger2 = new YoInteger("var2", "descriptionTest", registry);
      YoRegistry newRegistry = new YoRegistry("newRegistry");
      YoInteger duplicate = yoInteger2.duplicate(newRegistry);
      assertEquals(yoInteger2.getName(), duplicate.getName());
      assertEquals(yoInteger2.getDescription(), duplicate.getDescription());
      assertEquals(yoInteger2.getLowerBound(), duplicate.getLowerBound(), EPSILON);
      assertEquals(yoInteger2.getUpperBound(), duplicate.getUpperBound(), EPSILON);
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
