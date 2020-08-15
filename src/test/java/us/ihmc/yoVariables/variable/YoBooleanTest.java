package us.ihmc.yoVariables.variable;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;

public class YoBooleanTest
{
   private YoBoolean yoBoolean;
   private YoRegistry registry;
   private static final double EPSILON = 1e-10;

   @BeforeEach
   public void setUp()
   {
      registry = new YoRegistry("testRegistry");
      yoBoolean = new YoBoolean("booleanVariable", registry);
   }

   @AfterEach
   public void tearDown()
   {
      yoBoolean = null;
      registry = null;
   }

   @Test // timeout=300000
   public void testBooleanYoVariable()
   {
      assertEquals("booleanVariable", yoBoolean.getName());
      assertEquals("testRegistry", yoBoolean.getRegistry().getName());
      assertEquals(false, yoBoolean.getBooleanValue());
   }

   @Test // timeout=300000
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

   @Test // timeout=300000
   public void testGetAndSetBooleanYoVariable()
   {
      yoBoolean.set(false);
      assertFalse(yoBoolean.getBooleanValue());

      yoBoolean.set(true);
      assertTrue(yoBoolean.getBooleanValue());
   }

   @Test // timeout=300000
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

   @Test // timeout=300000
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

      for (int counter = 0; counter < 100; counter++)
      {
         testRandom = rng.nextDouble();
         yoBoolean.setValueFromDouble(testRandom);
         if (testRandom >= 0.5)
            assertTrue(yoBoolean.getBooleanValue());
         else
            assertFalse(yoBoolean.getBooleanValue());
      }
   }

   @Test // timeout=300000
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

   @Test // timeout=300000
   public void testToString()
   {
      yoBoolean.set(false);
      assertEquals(yoBoolean.toString(), "booleanVariable: false");
      yoBoolean.set(true);
      assertEquals(yoBoolean.toString(), "booleanVariable: true");
   }

   @Test // timeout=300000
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

   @Test // timeout=300000
   public void testYoVariableType()
   {
      assertEquals(yoBoolean.getType(), YoVariableType.BOOLEAN);
   }

   @Test // timeout=300000
   public void testDuplicate()
   {
      YoRegistry newRegistry = new YoRegistry("newTestRegistry");
      YoBoolean val, testVal;

      yoBoolean.set(true);
      val = yoBoolean.duplicate(newRegistry);
      testVal = (YoBoolean) newRegistry.getVariables().get(0);
      assertEquals(yoBoolean.getBooleanValue(), val.getBooleanValue());
      assertEquals(yoBoolean.getBooleanValue(), testVal.getBooleanValue());
      assertTrue(val.getRegistry().equals(newRegistry));
      assertTrue(val.getRegistry().equals(testVal.getRegistry()));
      newRegistry = null;
   }

   @Test // timeout=300000
   public void testProviderValue()
   {
      assertEquals(yoBoolean.getBooleanValue(), yoBoolean.getValue());
      yoBoolean.set(true);
      assertEquals(yoBoolean.getBooleanValue(), yoBoolean.getValue());
   }
}