package us.ihmc.yoVariables.dataBuffer;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;
import us.ihmc.yoVariables.variable.YoBoolean;

public class DataBufferTest
{
   private final int testBufferSize = 100;
   
   private enum EnumYoVariableTestEnums
   {
      ONE, TWO;
   }

   private YoEnum<EnumYoVariableTestEnums> yoEnum;
   private YoDouble yoDouble;
   private YoBoolean yoBoolean;
   private YoInteger yoInteger;
   private YoVariableRegistry registry;
   private DataBuffer dataBuffer = new DataBuffer(testBufferSize);
   
   private YoDouble a, b, c;
   private DataBufferEntry aBuffer, bBuffer, cBuffer;

   public DataBufferTest()
   {
      
   }
   
   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      yoDouble = new YoDouble("yoDouble", registry);
      yoBoolean = new YoBoolean("yoBoolean", registry);
      yoInteger = new YoInteger("yoInteger", registry);
      yoEnum = new YoEnum<EnumYoVariableTestEnums>("yoEnum", registry, EnumYoVariableTestEnums.class);
      
      a = new YoDouble("a_arm", registry);
      b = new YoDouble("b_arm", registry);
      c = new YoDouble("c_arm", registry);
      
      aBuffer = new DataBufferEntry(a, testBufferSize);
      bBuffer = new DataBufferEntry(b, testBufferSize);
      cBuffer = new DataBufferEntry(c, testBufferSize);
   }

   @After
   public void tearDown()
   {
      yoDouble = null;
      registry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetBufferSize()
   {
      int testBufferSize = dataBuffer.getBufferSize();
      int expectedBufferSize = testBufferSize;
      assertTrue(expectedBufferSize == testBufferSize);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMaxBufferSize()
   {
      int expectedMaxBufferSize = 16384;
      int testMaxBufferSize = dataBuffer.getMaxBufferSize();
      assertTrue(expectedMaxBufferSize == testMaxBufferSize);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetAndSetWrapBuffer()
   {
      dataBuffer.setWrapBuffer(false);
      boolean testBoolean = dataBuffer.getWrapBuffer();
      assertFalse(testBoolean);
      dataBuffer.setWrapBuffer(true);
      testBoolean = dataBuffer.getWrapBuffer();
      assertTrue(testBoolean); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddAndGetEntry()
   {
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(yoDouble, testBufferSize);
      DataBufferEntry booleanDataBufferEntryTest = new DataBufferEntry(yoBoolean, testBufferSize);
      DataBufferEntry integerDataBufferEntryTest = new DataBufferEntry(yoInteger, testBufferSize);
      DataBufferEntry enumDataBufferEntryTest = new DataBufferEntry(yoEnum, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      dataBuffer.addEntry(booleanDataBufferEntryTest);
      dataBuffer.addEntry(integerDataBufferEntryTest);
      dataBuffer.addEntry(enumDataBufferEntryTest);
      
      DataBufferEntry testEntryReceivedViaString = dataBuffer.getEntry("yoDouble");
      DataBufferEntry testEntryReceivedViaVariableName = dataBuffer.getEntry(yoDouble);
      assertEquals(doubleDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(doubleDataBufferEntryTest, testEntryReceivedViaVariableName);
      
      testEntryReceivedViaString = dataBuffer.getEntry("yoBoolean");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(yoBoolean);
      assertEquals(booleanDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(booleanDataBufferEntryTest, testEntryReceivedViaVariableName);
      
      testEntryReceivedViaString = dataBuffer.getEntry("yoInteger");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(yoInteger);
      assertEquals(integerDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(integerDataBufferEntryTest, testEntryReceivedViaVariableName);
      
      testEntryReceivedViaString = dataBuffer.getEntry("yoEnum");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(yoEnum);
      assertEquals(enumDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(enumDataBufferEntryTest, testEntryReceivedViaVariableName);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddNewEntry() throws DataBuffer.RepeatDataBufferEntryException
   {
      dataBuffer.addVariable(yoDouble, testBufferSize);
      dataBuffer.addVariable(yoBoolean, testBufferSize);
      dataBuffer.addVariable(yoInteger, testBufferSize);
      dataBuffer.addVariable(yoEnum, testBufferSize);
      
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(yoDouble, testBufferSize);
      DataBufferEntry booleanDataBufferEntryTest = new DataBufferEntry(yoBoolean, testBufferSize);
      DataBufferEntry integerDataBufferEntryTest = new DataBufferEntry(yoInteger, testBufferSize);
      DataBufferEntry enumDataBufferEntryTest = new DataBufferEntry(yoEnum, testBufferSize);
      
      assertTrue(doubleDataBufferEntryTest.getVariable() == dataBuffer.getEntry(yoDouble).getVariable());
      assertTrue(booleanDataBufferEntryTest.getVariable() == dataBuffer.getEntry(yoBoolean).getVariable());
      assertTrue(integerDataBufferEntryTest.getVariable() == dataBuffer.getEntry(yoInteger).getVariable());
      assertTrue(enumDataBufferEntryTest.getVariable() == dataBuffer.getEntry(yoEnum).getVariable());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddVariable() throws DataBuffer.RepeatDataBufferEntryException
   {
      dataBuffer.addVariable(yoDouble);
      dataBuffer.addVariable(yoBoolean);
      dataBuffer.addVariable(yoInteger);
      dataBuffer.addVariable(yoEnum);
      
      assertTrue(yoDouble == dataBuffer.getEntry(yoDouble).getVariable());
      assertTrue(yoBoolean == dataBuffer.getEntry(yoBoolean).getVariable());
      assertTrue(yoInteger == dataBuffer.getEntry(yoInteger).getVariable());
      assertTrue(yoEnum == dataBuffer.getEntry(yoEnum).getVariable());

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddVariableWithArrayList() throws DataBuffer.RepeatDataBufferEntryException
   {
      ArrayList<YoVariable<?>> arrayListToBeAdded = new ArrayList<YoVariable<?>>();
      arrayListToBeAdded.add(yoDouble);
      arrayListToBeAdded.add(yoBoolean);
      arrayListToBeAdded.add(yoInteger);
      arrayListToBeAdded.add(yoEnum);
      
      dataBuffer.addVariables(arrayListToBeAdded);
      
      assertTrue(yoDouble == dataBuffer.getEntry(yoDouble).getVariable());
      assertTrue(yoBoolean == dataBuffer.getEntry(yoBoolean).getVariable());
      assertTrue(yoInteger == dataBuffer.getEntry(yoInteger).getVariable());
      assertTrue(yoEnum == dataBuffer.getEntry(yoEnum).getVariable());
      
   }
   
   //add dataBuffer listener?

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariablesThatContain() throws DataBuffer.RepeatDataBufferEntryException
   {
      YoDouble yoVariable123456789 = new YoDouble("123456789", registry);
      YoDouble yoVariable12345678 = new YoDouble("12345678", registry);
      YoDouble yoVariable1234567 = new YoDouble("1234567", registry);
      YoDouble yoVariable123456 = new YoDouble("123456", registry);
      YoDouble yoVariable12345 = new YoDouble("12345", registry);
      YoDouble yoVariable1234 = new YoDouble("1234", registry);
      YoDouble yoVariable123 = new YoDouble("123", registry);
      YoDouble yoVariable12 = new YoDouble("12", registry);
      YoDouble yoVariable1 = new YoDouble("1", registry);
      
      ArrayList<YoVariable<?>> currentlyMatched = new ArrayList<YoVariable<?>>();
      
      currentlyMatched.add(yoVariable123456789);
      currentlyMatched.add(yoVariable12345678);
      currentlyMatched.add(yoVariable1234567);
      currentlyMatched.add(yoVariable123456);
      currentlyMatched.add(yoVariable12345);
      currentlyMatched.add(yoVariable1234);
      currentlyMatched.add(yoVariable123);
      currentlyMatched.add(yoVariable12);
      currentlyMatched.add(yoVariable1);

      
      assertTrue(1 == dataBuffer.getVariablesThatContain("123456789", true, currentlyMatched).size());
      assertTrue(2 == dataBuffer.getVariablesThatContain("12345678", true, currentlyMatched).size());
      assertTrue(3 == dataBuffer.getVariablesThatContain("1234567", true, currentlyMatched).size());
      assertTrue(4 == dataBuffer.getVariablesThatContain("123456", true, currentlyMatched).size());
      assertTrue(5 == dataBuffer.getVariablesThatContain("12345", true, currentlyMatched).size());
      assertTrue(6 == dataBuffer.getVariablesThatContain("1234", true, currentlyMatched).size());
      assertTrue(7 == dataBuffer.getVariablesThatContain("123", true, currentlyMatched).size());
      assertTrue(8 == dataBuffer.getVariablesThatContain("12", true, currentlyMatched).size());
      assertTrue(9 == dataBuffer.getVariablesThatContain("1", true, currentlyMatched).size());
      assertTrue(null == dataBuffer.getVariablesThatContain("1234567890", true, currentlyMatched));
      assertTrue(null == dataBuffer.getVariablesThatContain("987654321", true, currentlyMatched));
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariablesThatStartWith() throws DataBuffer.RepeatDataBufferEntryException
   {
      
      YoDouble yoVariable1 = new YoDouble("doy", registry);
      YoDouble yoVariable2= new YoDouble("Dog", registry);
      YoDouble yoVariable3 = new YoDouble("bar", registry);
      
      dataBuffer.addVariable(yoDouble);
      dataBuffer.addVariable(yoBoolean);
      dataBuffer.addVariable(yoInteger);
      dataBuffer.addVariable(yoEnum);
      dataBuffer.addVariable(yoVariable1);
      dataBuffer.addVariable(yoVariable2);
      dataBuffer.addVariable(yoVariable3);
      
      assertTrue(3 == dataBuffer.getVariablesThatStartWith("d", false).size());
      assertTrue(2 == dataBuffer.getVariablesThatStartWith("d").size());
      assertTrue(2 == dataBuffer.getVariablesThatStartWith("b").size());
      
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetEntries() throws DataBuffer.RepeatDataBufferEntryException
   {
      ArrayList<DataBufferEntry> expectedDataEntries = new ArrayList<DataBufferEntry>();
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(yoDouble, testBufferSize);
      DataBufferEntry booleanDataBufferEntryTest = new DataBufferEntry(yoBoolean, testBufferSize);
      DataBufferEntry integerDataBufferEntryTest = new DataBufferEntry(yoInteger, testBufferSize);
      DataBufferEntry enumDataBufferEntryTest = new DataBufferEntry(yoEnum, testBufferSize);
      
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      dataBuffer.addEntry(booleanDataBufferEntryTest);
      dataBuffer.addEntry(integerDataBufferEntryTest);
      dataBuffer.addEntry(enumDataBufferEntryTest);
      
      expectedDataEntries.add(doubleDataBufferEntryTest);
      expectedDataEntries.add(booleanDataBufferEntryTest);
      expectedDataEntries.add(integerDataBufferEntryTest);
      expectedDataEntries.add(enumDataBufferEntryTest);
      
      assertEquals(expectedDataEntries, dataBuffer.getEntries());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariables() throws DataBuffer.RepeatDataBufferEntryException
   {
      dataBuffer.addVariable(yoDouble, testBufferSize);
      dataBuffer.addVariable(yoBoolean, testBufferSize);
      dataBuffer.addVariable(yoInteger, testBufferSize);
      dataBuffer.addVariable(yoEnum, testBufferSize);
      
      ArrayList<YoVariable<?>> expectedArrayOfVariables = new ArrayList<YoVariable<?>>();
      expectedArrayOfVariables.add(yoDouble);
      expectedArrayOfVariables.add(yoBoolean);
      expectedArrayOfVariables.add(yoInteger);
      expectedArrayOfVariables.add(yoEnum);
      
      ArrayList<YoVariable<?>> actualArrayOfVariables = dataBuffer.getAllVariables();
      
      for(int i = 0; i < actualArrayOfVariables.size(); i++)
      {
         assertTrue(expectedArrayOfVariables.contains(actualArrayOfVariables.get(i)));
      }
 
   }
   
/*   
   //testGetVars(String [], String[])
   
   //testGetVarsFromGroup(String varGroupName, VarGroupList varGroupList)
   
   //setMaxBufferSize
   
   
//   @Test(timeout=300000)
//   public void testResetDataBuffer() throws RepeatDataBufferEntryException
//   {
//      dataBuffer.addNewEntry(yoDouble, testBufferSize);
//      dataBuffer.addNewEntry(yoBoolean, testBufferSize);
//      dataBuffer.addNewEntry(yoInteger, testBufferSize);
//      dataBuffer.addNewEntry(yoEnum, testBufferSize);
//      
//      dataBuffer.resetDataBuffer();
//      
//      int dataBufferSize = dataBuffer.getBufferSize();
//      .println(dataBufferSize);
//      assertTrue(0 == dataBufferSize);
//
//   }
   
   
//   @Test(timeout=300000)
//   public void testClearAll() throws RepeatDataBufferEntryException
//   {
//      
//       dataBuffer.addNewEntry(yoDouble, testBufferSize);
//       dataBuffer.addNewEntry(yoBoolean, testBufferSize);
//       dataBuffer.addNewEntry(yoInteger, testBufferSize);
//       dataBuffer.addNewEntry(yoEnum, testBufferSize);
//       
//       dataBuffer.clearAll(testBufferSize);
//       
//       int dataBufferSize = dataBuffer.getBufferSize();
//       .println(dataBufferSize);
//       assertTrue(0 == dataBufferSize);
//      
//   }
  */   

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testEmptyBufferIncreaseBufferSize()
   {
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize * 2;
            
      dataBuffer.changeBufferSize(newBufferSize);
//      .println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testEmptyBufferDecreaseBufferSize()
   {
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize/2;
            
      dataBuffer.changeBufferSize(newBufferSize);
//      .println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testEnlargeBufferSize()
   {
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(yoDouble, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize * 2;
            
      dataBuffer.changeBufferSize(newBufferSize);
//      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDecreaseBufferSize()
   {
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(yoDouble, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize/2;
            
      dataBuffer.changeBufferSize(newBufferSize);
//      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize()); 
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testTick()
   {
      int numberOfTicksAndUpdates = 20;
      for (int i=0; i<numberOfTicksAndUpdates; i++)
      {
         dataBuffer.tickAndUpdate();
      }
      
      dataBuffer.gotoInPoint();
      
      int expectedIndex = 0;
      while(dataBuffer.getIndex() < dataBuffer.getBufferInOutLength()-1)
      {
         assertEquals(expectedIndex, dataBuffer.getIndex());
         boolean rolledOver = dataBuffer.tick(1);
         assertFalse(rolledOver);
         expectedIndex++;
      }
      
      boolean rolledOver = dataBuffer.tick(1);
      assertTrue(rolledOver);
      expectedIndex = 0;
      assertEquals(expectedIndex, dataBuffer.getIndex());
      
      rolledOver = dataBuffer.tick(1);
      assertFalse(rolledOver);
      expectedIndex = 1;
      assertEquals(expectedIndex, dataBuffer.getIndex());

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIsIndexBetweenInAndOutPoint()
   {
      assertEquals(0, dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(0, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(0));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(1));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(-1));
      
      dataBuffer.tickAndUpdate();
      assertEquals(1, dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(1, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(0));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(1));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(-1));
      
      dataBuffer.tickAndUpdate();
      assertEquals(2, dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(2, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(0));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(2));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(3));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(-1));
      
      int numTicks = 20;
      for (int i=0; i<numTicks; i++)
      {
         dataBuffer.tickAndUpdate();
      }
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());

      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()-1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()+1));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getInPoint()-1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getInPoint()));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getInPoint()+1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getOutPoint()-1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getOutPoint()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getOutPoint()+1));
      
      dataBuffer.cropData();
      dataBuffer.gotoOutPoint();
      
      dataBuffer.setWrapBuffer(true);
      
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());
      
      numTicks = 7;
      
      for (int i=0; i<numTicks; i++)
      {
         dataBuffer.tickAndUpdate();
      }
      
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(numTicks - 1, dataBuffer.getOutPoint());
      assertEquals(numTicks, dataBuffer.getInPoint());
      
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()-1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()+1));
      
      dataBuffer.tickAndUpdate();
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getInPoint()-1);
      
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()-1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()+1));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetSafeToChangeIndex() //Luke Morris
   {
      boolean isSafe = dataBuffer.isSafeToChangeIndex();
      assertTrue(isSafe);
      dataBuffer.setSafeToChangeIndex(false);
      boolean isNowSafe = dataBuffer.isSafeToChangeIndex();
      assertFalse(isNowSafe);
      dataBuffer.setSafeToChangeIndex(true);
      boolean isFinallySafe = dataBuffer.isSafeToChangeIndex();
      assertTrue(isFinallySafe);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariablesTwo() //Luke Morris
   {
      ArrayList<YoVariable<?>> variables = dataBuffer.getVariables();
      //    return dataBuffer.toString();
      //    return variables.toString();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVars() //Luke Morris

   {
      dataBuffer.addEntry(aBuffer);
      dataBuffer.addEntry(bBuffer);
      dataBuffer.addEntry(cBuffer);

      String[] varNames = new String[3];
      varNames[0] = "a_arm";
      varNames[1] = "b_arm";
      varNames[2] = "c_arm";

      String[] aNames = new String[1];
      aNames[0] = "a_arm";

      String[] allRegularExpressions = { ".*" };
      String[] cRegularExpressions = { "c.*" };

      ArrayList<YoVariable<?>> both = dataBuffer.getVars(varNames, allRegularExpressions);

      assertTrue(both.contains(a));
      assertTrue(both.contains(b));
      assertTrue(both.contains(c));

      ArrayList<YoVariable<?>> justNames = dataBuffer.getVars(varNames, null);

      assertTrue(justNames.contains(a));
      assertTrue(justNames.contains(b));
      assertTrue(justNames.contains(c));

      ArrayList<YoVariable<?>> justA = dataBuffer.getVars(aNames, null);

      assertTrue(justA.contains(a));
      assertFalse(justA.contains(b));
      assertFalse(justA.contains(c));

      ArrayList<YoVariable<?>> justRegExp = dataBuffer.getVars(null, allRegularExpressions);

      assertTrue(justRegExp.contains(a));
      assertTrue(justRegExp.contains(b));
      assertTrue(justRegExp.contains(c));

      ArrayList<YoVariable<?>> cRegExp = dataBuffer.getVars(null, cRegularExpressions);

      assertFalse(cRegExp.contains(a));
      assertFalse(cRegExp.contains(b));
      assertTrue(cRegExp.contains(c));

      ArrayList<YoVariable<?>> neither = dataBuffer.getVars(null, null);

      assertFalse(neither.contains(a));
      assertFalse(neither.contains(b));
      assertFalse(neither.contains(c));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetMaxBufferSize()

   {
      int minBuffer = 1;
      int maxBuffer = 10000000;
      int zeroBuffer = 0;
      int normalBuffer = 200;
      
      dataBuffer.setMaxBufferSize(minBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == minBuffer);
      
      dataBuffer.setMaxBufferSize(maxBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == maxBuffer);
      
      dataBuffer.setMaxBufferSize(zeroBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == zeroBuffer);
      
      dataBuffer.setMaxBufferSize(normalBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == normalBuffer);
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testClearAll()
   {
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
    public void testResetDataBuffer()
    {       
       dataBuffer.addEntry(aBuffer);
       dataBuffer.addEntry(bBuffer);
       dataBuffer.addEntry(cBuffer);
       
       ArrayList<YoVariable<?>> withVariables = dataBuffer.getVariables();
       
//       System.out.println(withVariables.size());
       assertTrue(withVariables.size() > 0);
       
       dataBuffer.resetDataBuffer();
       
       ArrayList<YoVariable<?>> resetVariables = dataBuffer.getVariables();
       
//       System.out.println(resetVariables.size());
//       assertTrue(resetVariables.size() == 0);
    }


}
