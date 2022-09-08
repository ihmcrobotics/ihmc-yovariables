package us.ihmc.yoVariables.variable;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

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
      assertEquals(1, registry.getVariables().size());
      assertTrue(yoEnum.getName().equals("yoEnum"));
      assertTrue(registry.findVariable("yoEnum").equals(yoEnum));
   }

   @Test // timeout=300000
   public void testConstructorWithDescription()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", "yoEnum with description", registry, EnumYoVariableTestEnums.class, false);
      assertFalse(yoEnum == null);
      assertEquals(1, registry.getVariables().size());
      assertTrue(yoEnum.getName().equals("yoEnum"));
      assertTrue(registry.findVariable("yoEnum").equals(yoEnum));
      assertTrue(yoEnum.getDescription().equals("yoEnum with description"));
   }

   @Test // timeout=300000
   public void testCreateNoDescription()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      assertFalse(yoEnum == null);
      assertEquals(1, registry.getVariables().size());
      assertTrue(yoEnum.getName().equals("yoEnum"));
      assertTrue(registry.findVariable("yoEnum").equals(yoEnum));
   }

   @Test // timeout=300000
   public void testCreateWithDescription()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", "yoEnum with description", registry, EnumYoVariableTestEnums.class, false);
      assertFalse(yoEnum == null);
      assertEquals(1, registry.getVariables().size());
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
      assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.set(EnumYoVariableTestEnums.TWO);
      assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
      yoEnum.set(EnumYoVariableTestEnums.ONE, false);
      assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.set(EnumYoVariableTestEnums.TWO, false);
      assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
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
      assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.setValueFromDouble(0.25);
      assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.setValueFromDouble(0.5);
      assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
      yoEnum.setValueFromDouble(1.0);
      assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
   }

   @Test // timeout=300000
   public void testSetValueAsDoubleOutOfBoundsJustIgnoresIt()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      EnumYoVariableTestEnums originalValue = yoEnum.getEnumValue();

      yoEnum.setValueFromDouble(2.0);
      assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());

      yoEnum.setValueFromDouble(-100.0);
      assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
   }

   @Test // timeout=300000
   public void testForNull()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", "", registry, EnumYoVariableTestEnums.class, true);
      yoEnum.setValueFromDouble(YoEnum.NULL_VALUE);
      assertEquals(yoEnum.getEnumValue(), null);
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
      assertEquals(yoEnum.getEnumValue(), null);
   }

   @Test // timeout=300000
   public void testGetValueAsDouble()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class, true);
      yoEnum.set(EnumYoVariableTestEnums.ONE);
      assertEquals(0.0, yoEnum.getValueAsDouble(), EPSILON);
      yoEnum.set(EnumYoVariableTestEnums.TWO);
      assertEquals(1.0, yoEnum.getValueAsDouble(), EPSILON);
      yoEnum.set(null);
      assertEquals(-1, yoEnum.getValueAsDouble(), EPSILON);
   }

   @Test // timeout=300000
   public void testToString()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      yoEnum.set(EnumYoVariableTestEnums.ONE);
      assertEquals("yoEnum: ONE", yoEnum.toString());
      yoEnum.set(EnumYoVariableTestEnums.TWO);
      assertEquals("yoEnum: TWO", yoEnum.toString());
   }

   @Test // timeout=300000
   public void testGetYoVariableType()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      assertEquals(YoVariableType.ENUM, yoEnum.getType());
   }

   @Test // timeout=300000
   public void testGetValueAsLongBitsAndSetValueFromLongBits()
   {
      boolean notifyListeners = true;
      long longValue = 1;
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class, true);
      yoEnum.setValueFromLongBits(longValue, notifyListeners);
      assertEquals(longValue, yoEnum.getValueAsLongBits());

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
      assertEquals(-1, yoEnum.getValueAsLongBits());
   }

   @Test // timeout=300000
   public void testGetEnumType()
   {
      enumValue = EnumYoVariableTestEnums.ONE;
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);
      yoEnum.set(enumValue);

      assertEquals(enumValue.getClass(), yoEnum.getEnumType());
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
      assertEquals(yoEnum2.isNullAllowed(), yoEnum.isNullAllowed());
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

   @Test
   public void testListener()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<>("anEnum", "", registry, EnumYoVariableTestEnums.class, true);
      yoEnum.set(null);

      MutableBoolean hasChanged = new MutableBoolean(false);
      yoEnum.addListener(v -> hasChanged.setTrue());

      for (EnumYoVariableTestEnums value : EnumYoVariableTestEnums.values())
      {
         yoEnum.set(value);
         assertTrue(hasChanged.isTrue());
         hasChanged.setFalse();
         yoEnum.set(value);
         assertFalse(hasChanged.isTrue());
         hasChanged.setFalse();
      }

      yoEnum.set(null);
      assertTrue(hasChanged.isTrue());
      hasChanged.setFalse();
      yoEnum.set(null);
      assertFalse(hasChanged.isTrue());
      hasChanged.setFalse();
   }

   enum EmptyEnum
   {

   }
}