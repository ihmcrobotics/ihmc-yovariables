package us.ihmc.yoVariables.variable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoEnumTest
{
   private enum EnumYoVariableTestEnums
   {
      ONE, TWO;
   }

   private EnumYoVariableTestEnums enumValue;

   private YoVariableRegistry registry = null;
   private static final double EPSILON = 1e-10;

   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
   }

   @After
   public void tearDown()
   {
      registry = null;
   }

   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructorNoDescription()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      assertFalse(yoEnum == null);
      assertTrue(registry.getAllVariables().size() == 1);
      assertTrue(yoEnum.getName().equals("yoEnum"));
      assertTrue(registry.getVariable("yoEnum").equals(yoEnum));
   }

   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructorWithDescription()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", "yoEnum with description",
            registry, EnumYoVariableTestEnums.class, false);
      assertFalse(yoEnum == null);
      assertTrue(registry.getAllVariables().size() == 1);
      assertTrue(yoEnum.getName().equals("yoEnum"));
      assertTrue(registry.getVariable("yoEnum").equals(yoEnum));
      assertTrue(yoEnum.getDescription().equals("yoEnum with description"));
   }

   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCreateNoDescription()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = YoEnum.create("yoEnum", EnumYoVariableTestEnums.class, registry);
      assertFalse(yoEnum == null);
      assertTrue(registry.getAllVariables().size() == 1);
      assertTrue(yoEnum.getName().equals("yoEnum"));
      assertTrue(registry.getVariable("yoEnum").equals(yoEnum));
   }

   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCreateWithDescription()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = YoEnum.create("yoEnum", "yoEnum with description",
            EnumYoVariableTestEnums.class, registry, false);
      assertFalse(yoEnum == null);
      assertTrue(registry.getAllVariables().size() == 1);
      assertTrue(yoEnum.getName().equals("yoEnum"));
      assertTrue(registry.getVariable("yoEnum").equals(yoEnum));
      
      assertTrue(yoEnum.getDescription().equals("yoEnum with description"));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetAndValueEquals()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class, true);
      yoEnum.set(EnumYoVariableTestEnums.ONE);
      assertTrue(yoEnum.valueEquals(EnumYoVariableTestEnums.ONE));
      yoEnum.set(EnumYoVariableTestEnums.TWO);
      assertTrue(yoEnum.valueEquals(EnumYoVariableTestEnums.TWO));

      yoEnum.set(null);
      assertTrue(yoEnum.valueEquals(null));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetAndGet()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      yoEnum.set(EnumYoVariableTestEnums.ONE);
      Assert.assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.set(EnumYoVariableTestEnums.TWO);
      Assert.assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
      yoEnum.set(EnumYoVariableTestEnums.ONE,false);
      Assert.assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.set(EnumYoVariableTestEnums.TWO,false);
      Assert.assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValues()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      EnumYoVariableTestEnums[] enumTypeArray = yoEnum.getEnumValues();
      assertEquals(2, enumTypeArray.length);
      assertEquals(EnumYoVariableTestEnums.ONE, enumTypeArray[0]);
      assertEquals(EnumYoVariableTestEnums.TWO, enumTypeArray[1]);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValueAsDoublePositiveNumber()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      yoEnum.setValueFromDouble(0.0);
      Assert.assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.setValueFromDouble(0.25);
      Assert.assertEquals(EnumYoVariableTestEnums.ONE, yoEnum.getEnumValue());
      yoEnum.setValueFromDouble(0.5);
      Assert.assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
      yoEnum.setValueFromDouble(1.0);
      Assert.assertEquals(EnumYoVariableTestEnums.TWO, yoEnum.getEnumValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValueAsDoubleOutOfBoundsJustIgnoresIt()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      EnumYoVariableTestEnums originalValue = yoEnum.getEnumValue();

      yoEnum.setValueFromDouble(2.0);
      Assert.assertEquals(originalValue, yoEnum.getEnumValue());

      yoEnum.setValueFromDouble(-100.0);
      Assert.assertEquals(originalValue, yoEnum.getEnumValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testForNull()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", "", registry,
            EnumYoVariableTestEnums.class, true);
      yoEnum.setValueFromDouble(YoEnum.NULL_VALUE);
      Assert.assertEquals(yoEnum.getEnumValue(), null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testNotAllowNull()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      yoEnum.set(null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAllowNull()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", "", registry,
            EnumYoVariableTestEnums.class, true);
      yoEnum.set(null);
      Assert.assertEquals(yoEnum.getEnumValue(), null);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueAsDouble()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class, true);
      yoEnum.set(EnumYoVariableTestEnums.ONE);
      Assert.assertEquals(0.0, yoEnum.getValueAsDouble(), EPSILON);
      yoEnum.set(EnumYoVariableTestEnums.TWO);
      Assert.assertEquals(1.0, yoEnum.getValueAsDouble(), EPSILON);
      yoEnum.set(null);
      Assert.assertEquals(-1, yoEnum.getValueAsDouble(), EPSILON);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testToString()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      yoEnum.set(EnumYoVariableTestEnums.ONE);
      Assert.assertEquals("yoEnum: ONE", yoEnum.toString());
      yoEnum.set(EnumYoVariableTestEnums.TWO);
      Assert.assertEquals("yoEnum: TWO", yoEnum.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueStringBufferWithNullValue()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", "", registry,
            EnumYoVariableTestEnums.class, true);
      yoEnum.set(null);

      StringBuffer valueBuffer = new StringBuffer();
      yoEnum.getValueString(valueBuffer);

      assertEquals("null", valueBuffer.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueStringBuffer()
   {
      StringBuffer valueBuffer = new StringBuffer();
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);

      yoEnum.set(EnumYoVariableTestEnums.ONE);
      yoEnum.getValueString(valueBuffer);
      assertEquals("ONE", valueBuffer.toString());

      yoEnum.set(EnumYoVariableTestEnums.TWO);
      yoEnum.getValueString(valueBuffer);
      assertEquals("ONETWO", valueBuffer.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetYoVariableType()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      Assert.assertEquals(YoVariableType.ENUM, yoEnum.getYoVariableType());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueAsLongBitsAndSetValueFromLongBits()
   {
      boolean notifyListeners = true;
      long longValue = 1;
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class, true);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetEnumType()
   {
      enumValue = EnumYoVariableTestEnums.ONE;
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      yoEnum.set(enumValue);

      Assert.assertEquals(enumValue.getClass(), yoEnum.getEnumType());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValueStringFromDouble()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class, true);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDuplicate()
   {
      YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry");
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      YoEnum<EnumYoVariableTestEnums> yoEnum2 = yoEnum.duplicate(newRegistry);

      assertTrue(yoEnum2.getName().equals(yoEnum.getName()));
      assertTrue(yoEnum2.getDescription().equals(yoEnum.getDescription()));
      assertTrue(yoEnum2.getEnumType().equals(yoEnum.getEnumType()));
      Assert.assertEquals(yoEnum2.getAllowNullValue(), yoEnum.getAllowNullValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValue()
   {
      YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry2");
      boolean notifyListeners = true;
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      YoEnum<EnumYoVariableTestEnums> yoEnum2 = new YoEnum<EnumYoVariableTestEnums>("yoEnum", newRegistry,
            EnumYoVariableTestEnums.class);

      enumValue = EnumYoVariableTestEnums.TWO;
      yoEnum2.set(enumValue);
      yoEnum.setValue(yoEnum2, notifyListeners);
      Assert.assertEquals(enumValue, yoEnum.getEnumValue());

      enumValue = EnumYoVariableTestEnums.ONE;
      yoEnum2.set(enumValue);
      yoEnum.setValue(yoEnum2, notifyListeners);
      Assert.assertEquals(enumValue, yoEnum.getEnumValue());
   }
	
	  
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testProviderValue()
   {
      YoEnum<EnumYoVariableTestEnums> yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry,
            EnumYoVariableTestEnums.class);
      YoEnum<EnumYoVariableTestEnums> yoEnum2 = new YoEnum<EnumYoVariableTestEnums>("yoEnum2", registry,
            EnumYoVariableTestEnums.class);
      
      yoEnum.set(EnumYoVariableTestEnums.ONE);
      yoEnum2.set(EnumYoVariableTestEnums.TWO);

      assertEquals(yoEnum.getEnumValue(), yoEnum.getValue());
      assertEquals(yoEnum2.getEnumValue(), yoEnum2.getValue());
   }
}