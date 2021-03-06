package us.ihmc.yoVariables.registry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableListTest
{

   @Test // timeout=300000
   public void testCommonUsage()
   {
      YoVariableList varList = new YoVariableList("listOne");
      YoRegistry registryOne = new YoRegistry("registryOne");
      YoRegistry registryTwo = new YoRegistry("registryTwo");

      YoBoolean booleanOne = new YoBoolean("booleanOne", registryOne);
      YoDouble doubleOne = new YoDouble("doubleOne", registryOne);
      varList.add(booleanOne);
      varList.add(doubleOne);

      YoBoolean booleanTwo = new YoBoolean("booleanTwo", registryTwo);
      YoDouble doubleTwo = new YoDouble("doubleTwo", registryTwo);
      varList.add(booleanTwo);
      varList.add(doubleTwo);

      // Here's the tricky case. Same name variable booleanOne but with a
      // different name space.
      // Design question is should varList.getVariable("booleanOne") return
      // the first one, both, or throw an exception?
      // Right now it returns the first one. This has problems with
      // graphGroups and such and should be fixed later
      YoBoolean repeatBooleanOne = new YoBoolean("booleanOne", registryTwo);
      varList.add(repeatBooleanOne);

      YoBoolean notIncluded = new YoBoolean("notIncluded", registryTwo);

      assertEquals("listOne", varList.getName());
      assertEquals(0, varList.indexOf(booleanOne));
      assertEquals(1, varList.indexOf(doubleOne));
      assertEquals(2, varList.indexOf(booleanTwo));
      assertEquals(3, varList.indexOf(doubleTwo));
      assertEquals(4, varList.indexOf(repeatBooleanOne));

      assertEquals(-1, varList.indexOf(notIncluded));

      assertTrue(varList.contains(booleanOne));
      assertTrue(varList.contains(doubleOne));
      assertTrue(varList.contains(booleanTwo));
      assertTrue(varList.contains(doubleTwo));
      assertTrue(varList.contains(repeatBooleanOne));
      assertFalse(varList.contains(notIncluded));

      assertThrows(IndexOutOfBoundsException.class, () -> varList.get(-1));
      assertTrue(booleanOne == varList.get(0));
      assertTrue(doubleOne == varList.get(1));
      assertTrue(booleanTwo == varList.get(2));
      assertTrue(doubleTwo == varList.get(3));
      assertTrue(repeatBooleanOne == varList.get(4));
      assertThrows(IndexOutOfBoundsException.class, () -> varList.get(5));

      assertTrue(varList.hasVariable("booleanOne"));
      assertFalse(varList.hasUniqueVariable("booleanOne"));
      assertTrue(varList.hasUniqueVariable("registryOne.booleanOne"));
      assertTrue(varList.hasUniqueVariable("doubleOne"));
      assertTrue(varList.hasUniqueVariable("registryOne.doubleOne"));
      assertTrue(varList.hasUniqueVariable("booleanTwo"));
      assertTrue(varList.hasUniqueVariable("registryTwo.booleanTwo"));
      assertTrue(varList.hasUniqueVariable("doubleTwo"));
      assertTrue(varList.hasUniqueVariable("registryTwo.doubleTwo"));
      assertTrue(varList.hasUniqueVariable("registryTwo.booleanOne"));

      assertFalse(varList.hasUniqueVariable("notIncluded"));
      assertFalse(varList.hasUniqueVariable("registryOne.doubleTwo"));

      assertTrue(booleanOne == varList.findVariable("booleanOne"));
      assertTrue(doubleOne == varList.findVariable("doubleOne"));
      assertTrue(booleanTwo == varList.findVariable("booleanTwo"));
      assertTrue(doubleTwo == varList.findVariable("doubleTwo"));
      assertTrue(booleanOne == varList.findVariable("registryOne.booleanOne"));
      assertTrue(doubleOne == varList.findVariable("registryOne.doubleOne"));
      assertTrue(booleanTwo == varList.findVariable("registryTwo.booleanTwo"));
      assertTrue(doubleTwo == varList.findVariable("registryTwo.doubleTwo"));

      assertTrue(repeatBooleanOne == varList.findVariable("registryTwo.booleanOne"));

      assertTrue(null == varList.findVariable("registryOne.doubleTwo"));
      assertTrue(null == varList.findVariable("notIncluded"));

   }

   @Test // timeout = 30000
   public void testGetPerformanceInLargeList()
   {
      // Test should take O(n) or O(n lg n) approximately. Was taking O(n^2)
      // with old implementation, which made data file reading really slow.
      // Automatic test somewhat tests the speed of this but need to run fast
      // so use small number.
      // Have tested manually by using numberOfVariables = 20000 and it takes
      // less than 5 seconds.

      YoRegistry rootRegistry = new YoRegistry("rootRegistry");
      YoRegistry registryOne = new YoRegistry("registryOne");
      YoRegistry registryTwo = new YoRegistry("registryTwo");
      YoRegistry registryThree = new YoRegistry("registryThree");

      rootRegistry.addChild(registryOne);
      registryOne.addChild(registryTwo);
      registryTwo.addChild(registryThree);

      YoDouble t = new YoDouble("t", registryThree);
      YoDouble time = new YoDouble("time", registryThree);
      t.set(1.1);
      time.set(2.2);

      int numberOfVariables = 4000;
      ArrayList<YoVariable> variables = new ArrayList<>();
      YoVariableList varList = new YoVariableList("test");

      for (int i = 0; i < numberOfVariables; i++)
      {
         YoDouble variableA = new YoDouble("variable" + i, registryThree);
         YoDouble variableB = new YoDouble("variable" + i, registryTwo);
         variableA.set(Math.random());
         variableB.set(Math.random());

         variables.add(variableA);
         variables.add(variableB);
         varList.add(variableA);
         varList.add(variableB);
      }

      assertEquals(variables.size(), varList.size());

      for (int i = 0; i < variables.size(); i++)
      {
         YoVariable yoVariable = variables.get(i);

         assertTrue(varList.hasUniqueVariable(yoVariable.getFullNameString()));
         assertTrue(yoVariable == varList.findVariable(yoVariable.getFullNameString()));
      }
   }

   @Test // timeout=300000
   public void testToString()
   {
      YoRegistry registry = new YoRegistry("registry");
      YoVariableList list = new YoVariableList("list");

      YoDouble a = new YoDouble("a", registry);
      YoDouble b = new YoDouble("b", registry);
      YoDouble c = new YoDouble("c", registry);

      list.add(a);
      list.add(b);
      list.add(c);

      assertEquals(list.getName() + ", variables:\n" + a.toString() + "\n" + b.toString() + "\n" + c.toString(), list.toString());

   }

   @Test // timeout=300000
   public void testAddVariables()
   {
      YoRegistry registry = new YoRegistry("registry");
      YoVariableList list = new YoVariableList("list");
      YoVariableList listTwo = new YoVariableList("listTwo");
      YoVariableList listThree = new YoVariableList("listThree");
      YoVariableList listFour = new YoVariableList("listFour");

      YoDouble a = new YoDouble("a", registry);
      YoDouble b = new YoDouble("b", registry);
      YoDouble c = new YoDouble("c", registry);

      //YoVariableList
      list.add(a);
      assertEquals(1, list.size());
      list.add(a); // Ignores if added twice.
      assertEquals(1, list.size());

      listTwo.add(b);
      listTwo.add(c);
      assertEquals(2, listTwo.size());

      list.addAll(listTwo);
      assertEquals(3, list.size());

      //YoVariable[]
      listThree.add(a);

      YoVariable[] array = new YoVariable[2];
      array[0] = b;
      array[1] = c;

      listThree.addAll(Arrays.asList(array));

      //ArrayList
      ArrayList<YoVariable> arrayList = new ArrayList<>();
      arrayList.add(b);
      arrayList.add(c);

      listFour.add(a);

      listFour.addAll(arrayList);

      //assertions
      for (int i = 0; i < list.size(); i++)
      {
         assertEquals(list.get(i), listThree.get(i));
         assertEquals(list.get(i), listFour.get(i));
      }
   }

   @Test // timeout=300000
   public void testCommonUsageTwo()
   {
      YoRegistry registry = new YoRegistry("registry");
      YoVariableList list = new YoVariableList("list");
      YoVariableList listTwo = new YoVariableList("list");
      YoVariableList listThree = new YoVariableList("list");

      assertTrue(list.isEmpty());

      YoDouble a = new YoDouble("a", registry);
      YoDouble b = new YoDouble("b", registry);
      YoDouble c = new YoDouble("c", registry);
      YoDouble f = new YoDouble("f", registry); //variable will not be added to list

      list.add(a);
      list.add(b);
      list.add(c);

      list.remove(b);
      list.remove(f); //attempt to remove variable not in list

      listTwo.add(a);
      listTwo.add(c);

      assertEquals(list.toString(), listTwo.toString());

      assertFalse(list.isEmpty());

      list.clear();
      assertEquals(list.toString(), listThree.toString());

      List<YoVariable> allVariables = listTwo.getVariables();

      for (int i = 0; i < listTwo.size(); i++)
      {
         assertEquals(allVariables.get(i).toString(), listTwo.get(i).toString());
      }
   }

   @Test // timeout=300000
   public void testGetMatchingVariables()
   {
      YoRegistry registry = new YoRegistry("registry");
      YoVariableList list = new YoVariableList("list");

      assertTrue(list.isEmpty());

      YoDouble a = new YoDouble("a_arm", registry);
      YoDouble b = new YoDouble("b_arm", registry);
      YoDouble c = new YoDouble("c_arm", registry);
      YoDouble f = new YoDouble("f_arm", registry); //variable will not be added to list

      list.add(a);
      list.add(b);
      list.add(c);

      String[] names = new String[2];
      names[0] = "a_arm";
      names[1] = "b_arm";

      String regularExpression = ".*";

      List<YoVariable> matchedAll = YoSearchTools.filterVariables(YoSearchTools.regularExpressionFilter(regularExpression), list);

      assertTrue(matchedAll.contains(a));
      assertTrue(matchedAll.contains(b));
      assertTrue(matchedAll.contains(c));
      assertFalse(matchedAll.contains(f));

      String regexpStartWithC = "c.*";
      List<YoVariable> matchedStartWithC = YoSearchTools.filterVariables(YoSearchTools.regularExpressionFilter(regexpStartWithC), list);

      assertFalse(matchedStartWithC.contains(a));
      assertFalse(matchedStartWithC.contains(b));
      assertTrue(matchedStartWithC.contains(c));
      assertFalse(matchedStartWithC.contains(f));

      List<YoVariable> namesOrStartWithC = YoSearchTools.filterVariables(YoSearchTools.regularExpressionFilter(regularExpression), list);

      assertTrue(namesOrStartWithC.contains(a));
      assertTrue(namesOrStartWithC.contains(b));
      assertTrue(namesOrStartWithC.contains(c));
      assertFalse(namesOrStartWithC.contains(f));

      // Return empty list when none match.
      List<YoVariable> matchedNameShouldBeEmpty = YoSearchTools.filterVariables(YoSearchTools.regularExpressionFilter("foo"), list);
      assertTrue(matchedNameShouldBeEmpty.isEmpty());
      matchedNameShouldBeEmpty = YoSearchTools.filterVariables(YoSearchTools.regularExpressionFilter("bar"), list);
      assertTrue(matchedNameShouldBeEmpty.isEmpty());
   }
}
