package us.ihmc.yoVariables.variable;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.robotics.Assert;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoLongTest
{
   private YoVariableRegistry registry;
   private Random random;
   private YoLong yoLong;
   private static final double EPSILON = 1e-10;

   @BeforeEach
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
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
         Assert.assertEquals(value, yoLong.getLongValue());
      }
   }

   @Test // timeout=300000
   public void testIncrementDecrementAddSubtract()
   {
      long value = random.nextLong();
      yoLong.set(value);

      yoLong.increment();
      Assert.assertEquals(value + 1, yoLong.getLongValue());

      yoLong.decrement();
      Assert.assertEquals(value, yoLong.getLongValue());

      yoLong.add(value);
      Assert.assertEquals(value * 2, yoLong.getLongValue());

      yoLong.subtract(value);
      Assert.assertEquals(value, yoLong.getLongValue());
   }

   @Test // timeout=300000
   public void testLargeValue()
   {
      long value = Long.MAX_VALUE - 2;
      yoLong.set(value);
      Assert.assertEquals(value, yoLong.getLongValue());
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
      Assert.assertEquals(0, yoLong.getLongValue());
      yoLong.set(0);
      Assert.assertEquals(0, yoLong.getLongValue());

      int value = random.nextInt() + 1;
      yoLong.set(value);
      Assert.assertEquals(value, yoLong.getLongValue());
   }

   @Test // timeout=300000
   public void testSetValueFromDouble()
   {
      double doubleValue = random.nextDouble();
      int intValue = (int) Math.round(doubleValue);
      boolean notifyListeners = true;
      yoLong.setValueFromDouble(doubleValue, notifyListeners);
      Assert.assertEquals(intValue, yoLong.getLongValue());
   }

   @Test // timeout=300000
   public void testGetValueAsDouble()
   {
      Assert.assertEquals(0.0, yoLong.getValueAsDouble(), EPSILON);
      long value = 15;
      yoLong.set(value);
      double result = yoLong.getValueAsDouble();
      assertEquals(15.0, result, EPSILON);
   }

   @Test // timeout=300000
   public void testToString()
   {
      Assert.assertEquals(yoLong.getName() + ": " + yoLong.getLongValue(), yoLong.toString());
   }

   @Test // timeout=300000
   public void testGetValueString()
   {
      Assert.assertEquals(0, yoLong.getLongValue());
      int value = random.nextInt();
      yoLong.set(value);
      StringBuffer stringBuffer = new StringBuffer();
      yoLong.getValueString(stringBuffer);
      assertEquals("" + value, stringBuffer.toString());
   }

   @Test // timeout=300000
   public void testGetValueStringFromDouble()
   {
      Assert.assertEquals(0, yoLong.getLongValue());
      double doubleValue = random.nextDouble();
      int value = (int) Math.round(doubleValue);
      yoLong.setValueFromDouble(doubleValue);
      StringBuffer stringBuffer = new StringBuffer();
      yoLong.getValueStringFromDouble(stringBuffer, doubleValue);
      assertEquals("" + value, stringBuffer.toString());
   }

   @Test // timeout=300000
   public void testGetYoVariableType()
   {
      Assert.assertEquals(YoVariableType.LONG, yoLong.getYoVariableType());
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
      Assert.assertEquals(longValue, yoLong.getValueAsLongBits());
   }

   @Test // timeout=300000
   public void testDuplicate()
   {
      YoLong yoLong2 = new YoLong("var2", "descriptionTest", registry);
      YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry");
      YoLong duplicate = yoLong2.duplicate(newRegistry);
      Assert.assertEquals(yoLong2.getName(), duplicate.getName());
      Assert.assertEquals(yoLong2.getDescription(), duplicate.getDescription());
      Assert.assertEquals(yoLong2.getManualScalingMin(), duplicate.getManualScalingMin(), EPSILON);
      Assert.assertEquals(yoLong2.getManualScalingMax(), duplicate.getManualScalingMax(), EPSILON);
   }

   @Test // timeout=300000
   public void testSetValue()
   {
      YoLong yoLong2 = new YoLong("var2", "descriptionTest", registry);
      boolean notifyListeners = true;
      yoLong.setValue(yoLong2, notifyListeners);
      Assert.assertEquals(yoLong2.getLongValue(), yoLong.getLongValue());
   }

   @Test // timeout = 300000
   public void testProviderValue()
   {
      yoLong.set(10L * Integer.MAX_VALUE);
      assertEquals(yoLong.getLongValue(), yoLong.getValue());
      yoLong.set(10L * Integer.MIN_VALUE);
      assertEquals(yoLong.getLongValue(), yoLong.getValue());

   }
}