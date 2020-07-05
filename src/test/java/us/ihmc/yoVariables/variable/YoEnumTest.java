package us.ihmc.yoVariables.variable;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.robotics.Assert;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoEnumTest
{
   private enum EnumYoVariableTestEnums
   {
      ONE, TWO;
   }

   private EnumYoVariableTestEnums enumValue;

   private YoRegistry registry = null;
   private static final double EPSILON = 1e-10;

   @BeforeEach
   public void setUp()
   {
      registry = new YoRegistry("testRegistry");
   }

   @AfterEach
   public void tearDown()
   {
      registry = null;
   }

   @Test // timeout=300000
   public void testConstructorNoDescription()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      assertFalse(yoEnum == null);
      assertTrue(registry.getVariables().size() == 1);
      assertTrue(yoEnum.getName().equals("yoEnum"));
      assertTrue(registry.findVariable("yoEnum").equals(yoEnum));
   }

   @Test // timeout=300000
   public void testConstructorWithDescription()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum",
                                                                                   "yoEnum with description",
                                                                                   registry,
                                                                                   EnumYoVariableTestEnums.class,
                                                                                   false);
      assertFalse(yoEnum == null);
      assertTrue(registry.getVariables().size() == 1);
      assertTrue(yoEnum.getName().equals("yoEnum"));
      assertTrue(registry.findVariable("yoEnum").equals(yoEnum));
      assertTrue(yoEnum.getDescription().equals("yoEnum with description"));
   }

   @Test // timeout=300000
   public void testCreateNoDescription()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = YoEnum.create("yoEnum", EnumYoVariableTestEnums.class, registry);
      assertFalse(yoEnum == null);
      assertTrue(registry.getVariables().size() == 1);
      assertTrue(yoEnum.getName().equals("yoEnum"));
      assertTrue(registry.findVariable("yoEnum").equals(yoEnum));
   }

   @Test // timeout=300000
   public void testCreateWithDescription()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = YoEnum.create("yoEnum", "yoEnum with description", EnumYoVariableTestEnums.class, registry, false);
      assertFalse(yoEnum == null);
      assertTrue(registry.getVariables().size() == 1);
      assertTrue(yoEnum.getName().equals("yoEnum"));
      assertTrue(registry.findVariable("yoEnum").equals(yoEnum));

      assertTrue(yoEnum.getDescription().equals("yoEnum with description"));
   }

   @Test // timeout=300000
   public void testSetAndValueEquals()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class, true);
      yoEnum.set(EnumYoVariableTestEnums.ONE);
      assertTrue(yoEnum.valueEquals(EnumYoVariableTestEnums.ONE));
      yoEnum.set(EnumYoVariableTestEnums.TWO);
      assertTrue(yoEnum.valueEquals(EnumYoVariableTestEnums.TWO));

      yoEnum.set(null);
      assertTrue(yoEnum.valueEquals(null));
   }

   @Test // timeout=300000
   public void testSetAndGet()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      yoEnum.set(EnumYoVariableTestEnums.ONE);
      Assert.assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.set(EnumYoVariableTestEnums.TWO);
      Assert.assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
      yoEnum.set(EnumYoVariableTestEnums.ONE, false);
      Assert.assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.set(EnumYoVariableTestEnums.TWO, false);
      Assert.assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
   }

   @Test // timeout=300000
   public void testGetValues()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      EnumYoVariableTestEnums[] enumTypeArray = yoEnum.getEnumValues();
      assertEquals(2, enumTypeArray.length);
      assertEquals(EnumYoVariableTestEnums.ONE, enumTypeArray[0]);
      assertEquals(EnumYoVariableTestEnums.TWO, enumTypeArray[1]);
   }

   @Test // timeout=300000
   public void testSetValueAsDoublePositiveNumber()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      yoEnum.setValueFromDouble(0.0);
      Assert.assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.setValueFromDouble(0.25);
      Assert.assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.setValueFromDouble(0.5);
      Assert.assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
      yoEnum.setValueFromDouble(1.0);
      Assert.assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
   }

   @Test // timeout=300000
   public void testSetValueAsDoubleOutOfBoundsJustIgnoresIt()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      EnumYoVariableTestEnums originalValue = yoEnum.getEnumValue();

      yoEnum.setValueFromDouble(2.0);
      Assert.assertEquals(originalValue, yoEnum.getEnumValue());

      yoEnum.setValueFromDouble(-100.0);
      Assert.assertEquals(originalValue, yoEnum.getEnumValue());
   }

   @Test // timeout=300000
   public void testForNull()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", "", registry, EnumYoVariableTestEnums.class, true);
      yoEnum.setValueFromDouble(YoEnum.NULL_VALUE);
      Assert.assertEquals(yoEnum.getEnumValue(), null);
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testNotAllowNull()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
         yoEnum.set(null);
      });
   }

   @Test // timeout=300000
   public void testAllowNull()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", "", registry, EnumYoVariableTestEnums.class, true);
      yoEnum.set(null);
      Assert.assertEquals(yoEnum.getEnumValue(), null);
   }

   @Test // timeout=300000
   public void testGetValueAsDouble()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class, true);
      yoEnum.set(EnumYoVariableTestEnums.ONE);
      Assert.assertEquals(0.0, yoEnum.getValueAsDouble(), EPSILON);
      yoEnum.set(EnumYoVariableTestEnums.TWO);
      Assert.assertEquals(1.0, yoEnum.getValueAsDouble(), EPSILON);
      yoEnum.set(null);
      Assert.assertEquals(-1, yoEnum.getValueAsDouble(), EPSILON);
   }

   @Test // timeout=300000
   public void testToString()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      yoEnum.set(EnumYoVariableTestEnums.ONE);
      Assert.assertEquals("yoEnum: ONE", yoEnum.toString());
      yoEnum.set(EnumYoVariableTestEnums.TWO);
      Assert.assertEquals("yoEnum: TWO", yoEnum.toString());
   }

   @Test // timeout=300000
   public void testGetValueStringBufferWithNullValue()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", "", registry, EnumYoVariableTestEnums.class, true);
      yoEnum.set(null);

      StringBuffer valueBuffer = new StringBuffer();
      yoEnum.getValueString(valueBuffer);

      assertEquals("null", valueBuffer.toString());
   }

   @Test // timeout=300000
   public void testGetValueStringBuffer()
   {
      StringBuffer valueBuffer = new StringBuffer();
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);

      yoEnum.set(EnumYoVariableTestEnums.ONE);
      yoEnum.getValueString(valueBuffer);
      assertEquals("ONE", valueBuffer.toString());

      yoEnum.set(EnumYoVariableTestEnums.TWO);
      yoEnum.getValueString(valueBuffer);
      assertEquals("ONETWO", valueBuffer.toString());
   }

   @Test // timeout=300000
   public void testGetYoVariableType()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      Assert.assertEquals(YoVariableType.ENUM, yoEnum.getYoVariableType());
   }

   @Test // timeout=300000
   public void testGetValueAsLongBitsAndSetValueFromLongBits()
   {
      boolean notifyListeners = true;
      long longValue = 1;
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class, true);
      yoEnum.setValueFromLongBits(longValue, notifyListeners);
      Assert.assertEquals(longValue, yoEnum.getValueAsLongBits());

      try
      {
         longValue = 2;
         yoEnum.setValueFromLongBits(longValue);
         fail();
      }
      catch (RuntimeException rte)
      {
         //do nothing
      }

      yoEnum.set(null);
      Assert.assertEquals(-1, yoEnum.getValueAsLongBits());
   }

   @Test // timeout=300000
   public void testGetEnumType()
   {
      enumValue = EnumYoVariableTestEnums.ONE;
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      yoEnum.set(enumValue);

      Assert.assertEquals(enumValue.getClass(), yoEnum.getEnumType());
   }

   @Test // timeout=300000
   public void testGetValueStringFromDouble()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class, true);
      StringBuffer stringBuffer = new StringBuffer();
      double doubleValue;

      doubleValue = -1.0;
      yoEnum.getValueStringFromDouble(stringBuffer, doubleValue);
      assertTrue(stringBuffer.toString().equals("null"));

      doubleValue = 0.0;
      yoEnum.getValueStringFromDouble(stringBuffer, doubleValue);
      assertTrue(stringBuffer.toString().equals("null" + "ONE"));

      doubleValue = 0.49;
      yoEnum.getValueStringFromDouble(stringBuffer, doubleValue);
      assertTrue(stringBuffer.toString().equals("null" + "ONE" + "ONE"));

      doubleValue = 0.5;
      yoEnum.getValueStringFromDouble(stringBuffer, doubleValue);
      assertTrue(stringBuffer.toString().equals("null" + "ONE" + "ONE" + "TWO"));

      doubleValue = 1.0;
      yoEnum.getValueStringFromDouble(stringBuffer, doubleValue);
      assertTrue(stringBuffer.toString().equals("null" + "ONE" + "ONE" + "TWO" + "TWO"));

      try
      {
         doubleValue = 1.5;
         yoEnum.getValueStringFromDouble(stringBuffer, doubleValue);
         fail();
      }
      catch (RuntimeException rte)
      {
      }
   }

   @Test // timeout=300000
   public void testDuplicate()
   {
      YoRegistry newRegistry = new YoRegistry("newRegistry");
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      YoEnum<EnumYoVariableTestEnums> yoEnum2 = yoEnum.duplicate(newRegistry);

      assertTrue(yoEnum2.getName().equals(yoEnum.getName()));
      assertTrue(yoEnum2.getDescription().equals(yoEnum.getDescription()));
      assertTrue(yoEnum2.getEnumType().equals(yoEnum.getEnumType()));
      Assert.assertEquals(yoEnum2.getAllowNullValue(), yoEnum.getAllowNullValue());
   }

   @Test // timeout=300000
   public void testSetValue()
   {
      YoRegistry newRegistry = new YoRegistry("newRegistry2");
      boolean notifyListeners = true;
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      YoEnum<EnumYoVariableTestEnums> yoEnum2 = new YoEnum<>("yoEnum", newRegistry, EnumYoVariableTestEnums.class);

      enumValue = EnumYoVariableTestEnums.TWO;
      yoEnum2.set(enumValue);
      yoEnum.setValue(yoEnum2, notifyListeners);
      Assert.assertEquals(enumValue, yoEnum.getEnumValue());

      enumValue = EnumYoVariableTestEnums.ONE;
      yoEnum2.set(enumValue);
      yoEnum.setValue(yoEnum2, notifyListeners);
      Assert.assertEquals(enumValue, yoEnum.getEnumValue());
   }

   @Test // timeout = 300000
   public void testProviderValue()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      YoEnum<EnumYoVariableTestEnums> yoEnum2 = new YoEnum<>("yoEnum2", registry, EnumYoVariableTestEnums.class);

      yoEnum.set(EnumYoVariableTestEnums.ONE);
      yoEnum2.set(EnumYoVariableTestEnums.TWO);

      assertEquals(yoEnum.getEnumValue(), yoEnum.getValue());
      assertEquals(yoEnum2.getEnumValue(), yoEnum2.getValue());
   }

   @Test // expected = RuntimeException.class, timeout = 1000
   public void testStringBasedAccessNullConstant()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         String[] constants = {"A", "B", "C", null, "E", "F", "G", "H"};

         YoRegistry registry = new YoRegistry("test");

         new YoEnum<>("constantDefault", "", registry, false, constants);
      });

   }

   @Test // timeout = 1000
   public void testEmptyConstantList()
   {
      String[] constants = {};
      YoRegistry registry = new YoRegistry("test");

      YoEnum<?> stringConstructor = new YoEnum<>("stringConstructor", "", registry, true, constants);
      YoEnum<?> enumConstructor = new YoEnum<>("enumConsturctor", "", registry, EmptyEnum.class, true);

      assertEquals(null, enumConstructor.getEnumValue());
      assertEquals("null", stringConstructor.getStringValue());
   }

   @Test // expected = RuntimeException.class, timeout = 1000
   public void testEmptyConstantListNotNull()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoRegistry registry = new YoRegistry("test");

         new YoEnum<>("enumConsturctor", "", registry, EmptyEnum.class, false);
      });
   }

   @Test // expected = RuntimeException.class, timeout = 1000
   public void testEmptyStringConstantListNotNull()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoRegistry registry = new YoRegistry("test");
         String[] constants = {};
         new YoEnum<>("stringConstructor", "", registry, false, constants);
      });
   }

   enum EmptyEnum
   {

   }
}