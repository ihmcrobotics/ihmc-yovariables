package us.ihmc.yoVariables.variable;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.ArrayList;
import java.util.NoSuchElementException;

import org.apache.commons.lang3.builder.ReflectionToStringBuilder;
import org.apache.commons.lang3.builder.ToStringStyle;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoTools;

public class YoVariableTest
{
   private YoVariable yoVariable = null;
   private YoRegistry registry = null;
   private ArrayList<TestVariableChangedListener> variableChangedListeners = null;

   public YoVariableTest()
   {
   }

   @BeforeEach
   public void setUp()
   {
      YoRegistry robotRegistry = new YoRegistry("robot");

      registry = new YoRegistry("testRegistry");
      robotRegistry.addChild(registry);

      yoVariable = new YoDouble("variableOne", registry);
      variableChangedListeners = new ArrayList<>();
   }

   @AfterEach
   public void tearDown()
   {
      yoVariable = null;
      registry = null;
      variableChangedListeners = null;
   }

   @Test // timeout=300000
   public void testFullNameEndsWith()
   {
      assertTrue(YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, "robot.testRegistry.variableOne"));
      assertTrue(YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, "testRegistry.variableOne"));
      assertTrue(YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, "variableOne"));
      assertTrue(!YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, "bot.testRegistry.variableOne"));
      assertTrue(!YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, ".testRegistry.variableOne"));
      assertTrue(!YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, "gistry.variableOne"));
      assertTrue(!YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, "ableOne"));
      assertTrue(!YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, "robot.testRegistr"));
      assertTrue(YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, "robot.testRegistry.VARIABLEONE"));
      assertFalse(YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, "Robot.testRegistry.variableOne"));
   }

   @Test // timeout=300000
   public void testValidVariable()
   {
      new YoDouble("foobar", "", null);
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testCantHaveADot()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         new YoDouble("foo.bar", "", null);
      });
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testCantHaveAComma()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         new YoDouble("foo,bar", "", null);
      });
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testCantHaveACarrot()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         new YoDouble("foo^bar", "", null);
      });
   }

   @Test // timeout=300000
   public void testCanHaveAClosingBracket()
   {
      new YoDouble("foo]bar", "", null);
   }

   @Test // timeout=300000
   public void testCanHaveAnOpeningBracket()
   {
      new YoDouble("foo[bar", "", null);
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testCantHaveABackSlash()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         new YoDouble("foo\\bar", "", null);
      });
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testCantHaveAQuote()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {

         new YoDouble("foo\"bar", "", null);
      });
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testCantHaveASpace()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         new YoDouble("foo bar", "", null);
      });
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testCantHaveASlash()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         new YoDouble("foo/bar", "", null);
      });
   }

   @Test // timeout=300000
   public void testGetBooleanValue()
   {
      YoBoolean booleanVariable = new YoBoolean("booleanVar", registry);
      booleanVariable.set(true);

      assert booleanVariable.getBooleanValue();
      booleanVariable.set(false);
      assert !booleanVariable.getBooleanValue();
   }

   @Test // timeout=300000
   public void testGetDescription()
   {
      YoDouble descrVariable = new YoDouble("booleanVar", "Description", registry);

      assertEquals(descrVariable.getDescription(), "Description");
      assertNotNull(yoVariable.getDescription());
   }

   @Test // timeout=300000
   public void testGetDoubleValue()
   {
      YoDouble doubleVariable = new YoDouble("doubleVar", registry);
      doubleVariable.set(15.6);

      assertEquals(doubleVariable.getDoubleValue(), 15.6, 1e-7);
   }

   private enum FooEnum
   {
      ONE, TWO, THREE;
   }

   @Test // timeout=300000
   public void testGetEnumValue()
   {
      YoEnum<FooEnum> enumVariable = new YoEnum<>("booleanVar", registry, FooEnum.class);
      enumVariable.set(FooEnum.TWO);

      assertEquals(enumVariable.getEnumValue(), FooEnum.TWO);
      assertFalse(enumVariable.getEnumValue() == FooEnum.ONE);
   }

   @Test // timeout=300000
   public void testGetFullNameWithNameSpace()
   {
      assertEquals(yoVariable.getFullNameString(), "robot.testRegistry.variableOne");
   }

   @Test // timeout=300000
   public void testGetIntegerValue()
   {
      YoInteger integerVariable = new YoInteger("integerVariable", registry);
      integerVariable.set(5);

      assertEquals(integerVariable.getIntegerValue(), 5);
   }

   @Test // timeout=300000
   public void testGetName()
   {
      assertEquals(yoVariable.getName(), "variableOne");
   }

   @Test // timeout=300000
   public void testGetYoRegistry()
   {
      YoRegistry registry = yoVariable.getRegistry();
      assertNotNull(registry);
      assertEquals(registry, this.registry);
      assertEquals(registry.findVariable(yoVariable.getName()), yoVariable);
   }

   @Test // timeout=300000
   public void testToString()
   {
      YoEnum<FooEnum> enumyoEnum = new YoEnum<>("enumYoVariable", registry, FooEnum.class);
      enumyoEnum.set(FooEnum.THREE);
      assertEquals("enumYoVariable: THREE", enumyoEnum.toString());

      YoInteger intyoVariable = new YoInteger("intYoVariable", registry);
      intyoVariable.set(1);
      assertEquals("intYoVariable: 1", intyoVariable.toString());

      YoDouble yoVariable = new YoDouble("doubleYoVariable", registry);
      yoVariable.set(0.0112);
      assertEquals("doubleYoVariable: 0.0112", yoVariable.toString());

      YoBoolean booleanyoBoolean = new YoBoolean("booleanYoVariable", registry);
      booleanyoBoolean.set(false);
      assertEquals("booleanYoVariable: false", booleanyoBoolean.toString());
   }

   @Test // timeout=300000
   public void testValueEquals()
   {
      YoBoolean booleanVariable = new YoBoolean("booleanVar", registry);
      YoDouble doubleVariable = new YoDouble("doubleVariable", registry);
      YoInteger intVariable = new YoInteger("intVariable", registry);
      YoEnum<FooEnum> enumVariable = new YoEnum<>("enumVariable", registry, FooEnum.class);

      booleanVariable.set(true);
      doubleVariable.set(1.4);
      intVariable.set(7);
      enumVariable.set(FooEnum.THREE);

      assert booleanVariable.valueEquals(true);
      assert doubleVariable.valueEquals(1.4);
      assert intVariable.valueEquals(7);
      assert enumVariable.valueEquals(FooEnum.THREE);

      booleanVariable.set(false);
      doubleVariable.set(4.5);
      intVariable.set(9);
      enumVariable.set(FooEnum.TWO);

      assert booleanVariable.valueEquals(false);
      assert doubleVariable.valueEquals(4.5);
      assert intVariable.valueEquals(9);
      assert enumVariable.valueEquals(FooEnum.TWO);

      assert !booleanVariable.valueEquals(true);
   }

   //   public void testYoVariable1()
   //   {
   //      // Did a lot of constructing already. Not testing constructors.
   //   }

   @Test // timeout=300000
   public void testNotifyVaribaleChangeListeners()
   {
      // create a bunch of Observers
      @SuppressWarnings("unused")
      int nObservers = 5;
      createVariableChangeListeners(5);

      // add them to the YoVariable
      yoVariable.removeListeners();
      addAllListenersToYoVariable();

      // create an observer that's not supposed to be notified.
      TestVariableChangedListener hearNoEvil = new TestVariableChangedListener();

      // make sure there's no event stored in the observers before we set
      for (TestVariableChangedListener listener : variableChangedListeners)
      {
         assertNull(listener.getLastVariableChanged());
      }

      assertNull(hearNoEvil.getLastVariableChanged());

      // now notify and check if the observers catch on

      yoVariable.notifyListeners();

      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         assertEquals(observer.getLastVariableChanged(), yoVariable);
      }

      // make sure hearNoEvil is unaware
      assertNull(hearNoEvil.getLastVariableChanged());
   }

   @Test // timeout=300000
   public void testAddVariableChangeListener()
   {
      // remove all observers, then add one new observer, and check if it can be removed without exceptions.
      yoVariable.removeListeners();
      TestVariableChangedListener listener = new TestVariableChangedListener();
      yoVariable.addListener(listener);
      yoVariable.removeListener(listener);
   }

   @Test // timeout=300000
   public void testRemoveAllVariableChangeListeners()
   {
      // create some observers, add them to yoVariable
      int nObservers = 5;
      createVariableChangeListeners(nObservers);
      addAllListenersToYoVariable();

      // let yoVariable notify observers. Assert that they got the event.

      yoVariable.notifyListeners();

      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         assertEquals(observer.getLastVariableChanged(), yoVariable);
      }

      // Remove and reset observers. Let yoVariable notify observers again. Assert that the observers didn't notice anything.
      yoVariable.removeListeners();
      resetAllObservers();
      yoVariable.notifyListeners();

      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         assertNull(observer.getLastVariableChanged());
      }
   }

   @Test // timeout=300000
   public void testRemoveObserver()
   {
      // create some observers, add them to yoVariable
      int nObservers = 5;
      createVariableChangeListeners(nObservers);
      addAllListenersToYoVariable();

      // let yoVariable notify observers. Assert that they got the event.
      yoVariable.notifyListeners();

      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         assertEquals(observer.getLastVariableChanged(), yoVariable);
      }

      // Remove and reset observers. Let yoVariable notify observers again. Assert that the observers didn't notice anything.
      for (YoVariableChangedListener observer : variableChangedListeners)
      {
         yoVariable.removeListener(observer);
      }

      resetAllObservers();
      yoVariable.notifyListeners();

      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         assertNull(observer.getLastVariableChanged());
      }
   }

   @Test // timeout=300000
   public void testRemoveObserverNonExistent1()
   {
      assertFalse(yoVariable.removeListener(new TestVariableChangedListener()));
   }

   @Test // timeout=300000,expected = NoSuchElementException.class
   public void testRemoveObserverNonExistent2()
   {
      createVariableChangeListeners(5);
      addAllListenersToYoVariable();
      assertFalse(yoVariable.removeListener(new TestVariableChangedListener()));
   }

   @Test // timeout=300000
   public void testRecursiveCompareYoVariables() throws IllegalArgumentException, IllegalAccessException, SecurityException, NoSuchFieldException
   {
      YoRegistry root0 = new YoRegistry("root");
      YoDouble variable01 = new YoDouble("variableOne", root0);
      YoDouble variable02 = new YoDouble("variableTwo", root0);

      YoRegistry root1 = new YoRegistry("root");
      YoDouble variable11 = new YoDouble("variableOne", root1);
      YoDouble variable12 = new YoDouble("variableTwo", root1);

      YoVariableComparer comparer = new YoVariableComparer(Integer.MAX_VALUE, Integer.MAX_VALUE);

      assertTrue(comparer.compare(root0, root1));
      assertTrue(comparer.compare(variable01, variable11));
      assertTrue(comparer.compare(variable02, variable12));

      variable01.set(4.4);

      // If a comparer returns false, next time it might return true since it stores list of everything that has been compared. So you need to be careful!
      assertFalse(comparer.compare(root0, root1));

      assertFalse(comparer.compare(variable01, variable11));
      assertTrue(comparer.compare(variable02, variable12));

      variable11.set(4.4);

      assertTrue(comparer.compare(root0, root1));
      assertTrue(comparer.compare(variable01, variable11));
      assertTrue(comparer.compare(variable02, variable12));

      variable02.set(99.9);
      assertTrue(comparer.compare(variable01, variable11));
      assertFalse(comparer.compare(variable02, variable12));
      assertFalse(comparer.compare(root0, root1));

      variable12.set(99.9);

      assertTrue(comparer.compare(variable01, variable11));
      assertTrue(comparer.compare(variable02, variable12));
      assertTrue(comparer.compare(root0, root1));

   }

   private void createVariableChangeListeners(int numberOfListeners)
   {
      for (int i = 0; i < numberOfListeners; i++)
      {
         TestVariableChangedListener listener = new TestVariableChangedListener();
         variableChangedListeners.add(listener);
      }
   }

   private void addAllListenersToYoVariable()
   {
      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         yoVariable.addListener(observer);
      }
   }

   private void resetAllObservers()
   {
      for (TestVariableChangedListener observer : variableChangedListeners)
      {
         observer.reset();
      }
   }

   /**
    * An VariableChangedListener that stores the most recent event and source, to be used for tests
    *
    * @author Twan Koolen
    */
   private class TestVariableChangedListener implements YoVariableChangedListener
   {
      private YoVariable lastVariableChanged = null;

      @Override
      public void changed(YoVariable v)
      {
         lastVariableChanged = v;
      }

      public YoVariable getLastVariableChanged()
      {
         return lastVariableChanged;
      }

      public void reset()
      {
         lastVariableChanged = null;
      }
   }

   private class YoVariableComparer
   {
      public YoVariableComparer(int maxDepth, int maxSize) throws NoSuchFieldException, SecurityException
      {
      }

      public boolean compare(YoDouble variable01, YoDouble variable11)
      {
         return ReflectionToStringBuilder.toString(variable01, ToStringStyle.NO_CLASS_NAME_STYLE)
                                         .equals(ReflectionToStringBuilder.toString(variable11, ToStringStyle.NO_CLASS_NAME_STYLE));

      }

      public boolean compare(YoRegistry root0, YoRegistry root1)
      {
         return ReflectionToStringBuilder.toString(root0, ToStringStyle.NO_CLASS_NAME_STYLE)
                                         .equals(ReflectionToStringBuilder.toString(root1, ToStringStyle.NO_CLASS_NAME_STYLE));
      }
   }
}
