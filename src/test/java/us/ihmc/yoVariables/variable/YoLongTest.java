package us.ihmc.yoVariables.variable;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;

public class YoLongTest
{
   private YoRegistry registry;
   private Random random;
   private YoLong yoLong;
   private static final double EPSILON = 1e-10;

   @BeforeEach
   public void setUp()
   {
      registry = new YoRegistry("testRegistry");
      random = new Random(1776L);
      yoLong = new YoLong("test", registry);
   }

   @AfterEach
   public void tearDown()
   {
      registry = null;
      yoLong = null;
   }

   @Test // timeout=300000
   public void testSetAndGet()
   {
      for (int i = 0; i < 100; i++)
      {
         long value = random.nextLong();
         boolean notify = random.nextBoolean();
         yoLong.set(value, notify);
         assertEquals(value, yoLong.getLongValue());
      }
   }

   @Test // timeout=300000
   public void testIncrementDecrementAddSubtract()
   {
      long value = random.nextLong();
      yoLong.set(value);

      yoLong.increment();
      assertEquals(value + 1, yoLong.getLongValue());

      yoLong.decrement();
      assertEquals(value, yoLong.getLongValue());

      yoLong.add(value);
      assertEquals(value * 2, yoLong.getLongValue());

      yoLong.subtract(value);
      assertEquals(value, yoLong.getLongValue());
   }

   @Test // timeout=300000
   public void testLargeValue()
   {
      long value = Long.MAX_VALUE - 2;
      yoLong.set(value);
      assertEquals(value, yoLong.getLongValue());
   }

   @Test // timeout=300000
   public void testValueEquals()
   {
      assertTrue(yoLong.valueEquals(0));

      long number = random.nextLong();
      yoLong.set(number);
      assertTrue(yoLong.valueEquals(number));
   }

   @Test // timeout=300000
   public void testSetFinal()
   {
      assertEquals(0, yoLong.getLongValue());
      yoLong.set(0);
      assertEquals(0, yoLong.getLongValue());

      int value = random.nextInt() + 1;
      yoLong.set(value);
      assertEquals(value, yoLong.getLongValue());
   }

   @Test // timeout=300000
   public void testSetValueFromDouble()
   {
      double doubleValue = random.nextDouble();
      int intValue = (int) Math.round(doubleValue);
      boolean notifyListeners = true;
      yoLong.setValueFromDouble(doubleValue, notifyListeners);
      assertEquals(intValue, yoLong.getLongValue());
   }

   @Test // timeout=300000
   public void testGetValueAsDouble()
   {
      assertEquals(0.0, yoLong.getValueAsDouble(), EPSILON);
      long value = 15;
      yoLong.set(value);
      double result = yoLong.getValueAsDouble();
      assertEquals(15.0, result, EPSILON);
   }

   @Test // timeout=300000
   public void testToString()
   {
      assertEquals(yoLong.getName() + ": " + yoLong.getLongValue(), yoLong.toString());
   }

   @Test // timeout=300000
   public void testGetYoVariableType()
   {
      assertEquals(YoVariableType.LONG, yoLong.getType());
   }

   @Test // timeout=300000
   public void testGetAndSetValueAsLongBits()
   {
      int value = 57;
      yoLong.set(value);
      long result = yoLong.getValueAsLongBits();
      assertEquals(value, result);

      long longValue = 12345;
      boolean notifyListeners = true;
      yoLong.setValueFromLongBits(longValue, notifyListeners);
      assertEquals(longValue, yoLong.getValueAsLongBits());
   }

   @Test // timeout=300000
   public void testDuplicate()
   {
      YoLong yoLong2 = new YoLong("var2", "descriptionTest", registry);
      YoRegistry newRegistry = new YoRegistry("newRegistry");
      YoLong duplicate = yoLong2.duplicate(newRegistry);
      assertEquals(yoLong2.getName(), duplicate.getName());
      assertEquals(yoLong2.getDescription(), duplicate.getDescription());
      assertEquals(yoLong2.getLowerBound(), duplicate.getLowerBound(), EPSILON);
      assertEquals(yoLong2.getUpperBound(), duplicate.getUpperBound(), EPSILON);
   }

   @Test // timeout = 300000
   public void testProviderValue()
   {
      yoLong.set(10L * Integer.MAX_VALUE);
      assertEquals(yoLong.getLongValue(), yoLong.getAsLong());
      yoLong.set(10L * Integer.MIN_VALUE);
      assertEquals(yoLong.getLongValue(), yoLong.getAsLong());

   }
}