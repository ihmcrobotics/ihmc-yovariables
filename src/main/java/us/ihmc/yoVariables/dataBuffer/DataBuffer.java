package us.ihmc.yoVariables.dataBuffer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.yoVariables.listener.RewoundListener;
import us.ihmc.yoVariables.registry.YoVariableList;
import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class DataBuffer implements DataBufferCommandsExecutor, ToggleKeyPointModeCommandExecutor, TimeDataHolder, DataEntryHolder
{
   private String timeVariableName = "t";

   private int inPoint = 0;

   private int index = 0;
   private int maxBufferSize = 16384;
   private int outPoint = 0;
   private List<RewoundListener> simulationRewoundListeners = null;
   private YoDouble t = null;
   private boolean wrapBuffer = false; // Default to Expand, not Wrap!  true;

   public KeyPoints keyPoints = new KeyPoints();
   private List<DataBufferListener> dataBufferListeners = new ArrayList<>();
   private int bufferSize;
   private ArrayList<DataBufferEntry> entries;
   private List<IndexChangedListener> indexChangedListeners;

   public List<ToggleKeyPointModeCommandListener> toggleKeyPointModeCommandListeners = new ArrayList<>();

   private boolean clearing = false;

   private boolean safeToManuallyChangeIndex = true;

   private final YoVariableList yoVariableList = new YoVariableList(getClass().getSimpleName());

   public DataBuffer(int bufferSize)
   {
      entries = new ArrayList<>();

      this.bufferSize = bufferSize;
   }

   @Override
   public void closeAndDispose()
   {
      dataBufferListeners.clear();
      dataBufferListeners = null;

      entries.clear();
      entries = null;

      index = -1;
   }

   public void setSafeToChangeIndex(boolean safe)
   {
      safeToManuallyChangeIndex = safe;
   }

   public boolean isSafeToChangeIndex()
   {
      return safeToManuallyChangeIndex;
   }

   public int getBufferSize()
   {
      return bufferSize;
   }

   public int getMaxBufferSize()
   {
      return maxBufferSize;
   }

   public boolean getWrapBuffer()
   {
      return wrapBuffer;
   }

   public void addEntry(DataBufferEntry entry)
   {
      if (entry.getDataLength() != bufferSize)
         throw new RuntimeException("entry.getDataLength() != this.bufferSize");

      entries.add(entry);
   }

   public DataBufferEntry addVariable(YoVariable newVariable, int nPoints)
   {
      yoVariableList.addVariable(newVariable);

      DataBufferEntry entry = new DataBufferEntry(newVariable, nPoints);
      addEntry(entry);

      if (newVariable.getName().equals("t"))
      {
         t = (YoDouble) newVariable;
      }

      return entry;
   }

   public void addVariable(YoVariable newVariable)
   {
      addVariable(newVariable, bufferSize);
   }

   public void addVariables(List<? extends YoVariable> variables)
   {
      entries.ensureCapacity(entries.size() + variables.size()); // do this first so that 'entries' will only have to grow once.

      for (int i = 0; i < variables.size(); i++)
      {
         YoVariable v = variables.get(i);

         //       System.out.println("Adding YoVariable: " + v);

         this.addVariable(v);
      }
   }

   public void addDataBufferListener(DataBufferListener dataBufferListener)
   {
      dataBufferListeners.add(dataBufferListener);
   }

   public List<YoVariable> getVariablesThatContain(String searchString, boolean caseSensitive, List<YoVariable> currentlyMatched)
   {
      ArrayList<YoVariable> ret = null;

      if (currentlyMatched != null)
      {
         if (!caseSensitive)
         {
            searchString = searchString.toLowerCase();
         }

         for (int i = 0; i < currentlyMatched.size(); i++)
         {
            YoVariable entry = currentlyMatched.get(i);

            if (entry.getName().toLowerCase().contains(searchString))
            {
               if (ret == null)
               {
                  ret = new ArrayList<>();
               }

               ret.add(entry);
            }
         }
      }

      return ret;
   }

   public List<YoVariable> getVariablesThatStartWith(String searchString)
   {
      ArrayList<YoVariable> ret = null;

      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         if (entry.getVariable().getName().startsWith(searchString))
         {
            if (ret == null)
            {
               ret = new ArrayList<>();
            }

            ret.add(entry.getVariable());
         }
      }

      return ret;
   }

   public DataBufferEntry getEntry(String name)
   {
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);
         YoVariable variable = entry.getVariable();

         if (YoTools.matchFullNameEndsWithCaseInsensitive(variable, name))
         {
            return entry;
         }
      }

      return null;
   }

   @Override
   public DataBufferEntry getEntry(YoVariable v)
   {
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         if (entry.getVariable() == v)
         {
            return entry;
         }
      }

      return null;
   }

   public List<DataBufferEntry> getEntries()
   {
      return entries;
   }

   public List<YoVariable> getVariables()
   {
      ArrayList<YoVariable> ret = new ArrayList<>(entries.size());

      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         ret.add(entry.getVariable());
      }

      return ret;
   }

   public List<YoVariable> getVars(String[] varNames, String[] regularExpressions)
   {
      YoVariableList tempList = new YoVariableList("temp");

      for (int i = 0; i < entries.size(); i++)
      {
         YoVariable var = entries.get(i).getVariable();

         tempList.addVariable(var);
      }

      List<YoVariable> variables = new ArrayList<>();
      if (varNames != null)
         Arrays.asList(varNames).forEach(varName -> variables.addAll(tempList.findVariables(varName)));
      if (regularExpressions != null)
         variables.addAll(YoSearchTools.filterVariables(YoSearchTools.regularExpressionFilter(regularExpressions), tempList));
      return variables;
   }

   /**
    * Sets the maximum size, in ticks, to which the buffer will expand. While nonsense values are not
    * explicitly checked for, they will not cause the buffer to shrink.
    *
    * @param newMaxBufferSize New max buffer size.
    */
   public void setMaxBufferSize(int newMaxBufferSize)
   {
      maxBufferSize = newMaxBufferSize;
   }

   /**
    * Enables or disables buffer wrapping in place of buffer expansion. By default the buffer will
    * expand until it reaches maxBufferSize at which point it will wrap to the beginning. When
    * wrapBuffer is enabled the buffer wraps to the beginning without attempting to expand.
    *
    * @param newWrapBuffer Enable or disable wrap buffer mode.
    */
   public void setWrapBuffer(boolean newWrapBuffer)
   {
      wrapBuffer = newWrapBuffer;
   }

   public void resetDataBuffer()
   {
      clearAll(getBufferSize());

      if (!clearing)
      {
         setInPoint(0);
         gotoInPoint();
         clearAll(getBufferSize());
         tickAndUpdate();
      }
      else
      {
         setInPoint(0);
         clearing = true;
      }
   }

   public void clearAll(int nPoints)
   {
      double[] blankData;

      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         blankData = new double[nPoints];
         entry.setData(blankData, nPoints);
      }

      bufferSize = nPoints;
   }

   public void changeBufferSize(int newBufferSize)
   {
      // if ((newBufferSize < 1) || (newBufferSize > maxBufferSize)) return;
      if (newBufferSize < bufferSize)
      {
         cropData(inPoint, (inPoint + newBufferSize - 1) % bufferSize);
      }
      else if (newBufferSize > bufferSize)
      {
         packData();
         enlargeBufferSize(newBufferSize);
      }

      // bufferSize = newBufferSize;
   }

   private void enlargeBufferSize(int newSize)
   {
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         entry.enlargeBufferSize(newSize);
      }

      bufferSize = newSize;
   }

   public void copyValuesThrough()
   {
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         entry.copyValueThrough();
      }
   }

   public int getBufferInOutLength()
   {
      int ret;

      if (outPoint > inPoint)
      {
         ret = outPoint - inPoint + 1;
      }
      else
      {
         ret = bufferSize - (inPoint - outPoint) + 1;
      }

      return ret;
   }

   public void packData()
   {
      packData(inPoint);
   }

   public void packData(int start)
   {
      if (start == 0)
         return;

      // If the start point is outside of the buffer abort.
      if (start <= 0 || start >= bufferSize)
      {
         return;
      }

      // Shift the data in each entry to begin with start.
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         entry.packData(start);
      }

      // Move the current index to its relative position in the new data set, if the index is outside of the buffer move to zero
      index = (index - start + bufferSize) % bufferSize;

      if (index < 0)
      {
         index = 0;
      }

      // Move the inPoint to the new beginning and the outPoint to the end
      inPoint = 0; // this.inPoint - start;
      outPoint = (outPoint - start + bufferSize) % bufferSize;

      // Move to the first tick
      this.tick(0);

      // +++++this.updateUI();
   }

   public void cropData()
   {
      if (inPoint != outPoint)
      {
         cropData(inPoint, outPoint);
      }
      else
      {
         cropData(inPoint, inPoint + 1);
      }
   }

   public void cropData(int start, int end)
   {
      // Abort if the start or end point is unreasonable
      if (start < 0 || end > bufferSize)
      {
         return; // -1; //SimulationConstructionSet.NUM_POINTS;
      }

      if (entries.isEmpty())
      {
         bufferSize = DataBufferEntry.computeBufferSizeAfterCrop(start, end, bufferSize);
      }

      // Step through the entries cropping and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);
         int retSize = entry.cropData(start, end);

         // If the result is a positive number store the new buffer size otherwise keep the original size
         if (retSize >= 0)
         {
            bufferSize = retSize;
         }
      }

      // Move the current index to its relative position after the resize
      index = (index - start + bufferSize) % bufferSize;

      // If the index is out of bounds move it to the beginning
      if (index < 0)
      {
         index = 0;
      }

      if (index >= bufferSize)
      {
         index = 0;
      }

      // Set the in point to the beginning and the out point to the end
      inPoint = 0;
      outPoint = bufferSize - 1;

      // Move to the first tick
      this.tick(0);

      // +++++this.updateUI();
   }

   public void cutData()
   {
      if (inPoint <= outPoint)
      {
         cutData(inPoint, outPoint);
      }
   }

   public void cutData(int start, int end)
   {
      // Abort if the start or end point is unreasonable
      if (start < 0 || end > bufferSize)
      {
         return;
      }

      if (entries.isEmpty())
      {
         bufferSize = DataBufferEntry.computeBufferSizeAfterCut(start, end, bufferSize);
      }

      // Step through the entries cutting and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);
         int retSize = entry.cutData(start, end);

         // If the result is a positive number store the new buffer size otherwise keep the original size
         if (retSize >= 0)
         {
            bufferSize = retSize;
         }
      }

      // Move the current index to its relative position after the resize
      index = (index - start + bufferSize) % bufferSize;

      // If the index is out of bounds move it to the beginning
      if (index < 0)
      {
         index = 0;
      }

      if (index >= bufferSize)
      {
         index = 0;
      }

      // Set the in point to the beginning and the out point to the end
      inPoint = 0;
      outPoint = start - 1;

      // Move to the first tick
      gotoOutPoint();
   }

   public void thinData(int keepEveryNthPoint)
   {
      packData();

      inPoint = 0;
      index = 0;

      if (bufferSize <= 2 * keepEveryNthPoint)
         return;

      // Step through the entries cutting and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);
         int retSize = entry.thinData(keepEveryNthPoint);

         // If the result is a positive number store the new buffer size otherwise keep the original size
         if (retSize >= 0)
         {
            bufferSize = retSize;
         }
      }

      outPoint = bufferSize - 1;

      gotoInPoint();
   }

   public double computeAverage(YoVariable variable)
   {
      DataBufferEntry entry = this.getEntry(variable);
      return entry.computeAverage();
   }

   @Override
   public int getInPoint()
   {
      return inPoint;
   }

   @Override
   public int getOutPoint()
   {
      return outPoint;
   }

   public void setInPoint()
   {
      setInPoint(index);
   }

   public void setOutPoint()
   {
      setOutPoint(index);
   }

   public void setInPoint(int in)
   {
      inPoint = in;
      keyPoints.trim(inPoint, outPoint);
   }

   public void setOutPoint(int out)
   {
      outPoint = out;
      keyPoints.trim(inPoint, outPoint);
   }

   public void setInOutPointFullBuffer()
   {
      inPoint = 0;
      outPoint = entries.get(0).getDataLength() - 1;
   }

   @Override
   public void gotoInPoint()
   {
      setIndex(inPoint);
   }

   @Override
   public void gotoOutPoint()
   {
      setIndex(outPoint);
   }

   public boolean atInPoint()
   {
      return index == inPoint;
   }

   public boolean atOutPoint()
   {
      return index == outPoint;
   }

   public void setKeyPoint()
   {
      keyPoints.setKeyPoint(index);
   }

   /**
    * Gets the KeyPoints in the cropped data
    *
    * @return The current KeyPoints as an ArrayList of Integer
    */
   public List<Integer> getKeyPoints()
   {
      // only return point in the cropped data
      return keyPoints.getPoints();
   }

   @Override
   public void setIndex(int index)
   {
      setIndex(index, true);
   }

   @Override
   public void setIndexButDoNotNotifySimulationRewoundListeners(int index)
   {
      this.setIndex(index, false);
   }

   private void setIndex(int index, boolean notifySimulationRewoundListeners)
   {
      if (safeToManuallyChangeIndex)
      {
         this.index = index;

         // if (this.index > this.getMaxIndex()) this.index = 0;
         if (this.index >= bufferSize)
         {
            this.index = 0;
         }
         else if (this.index < 0)
         {
            this.index = bufferSize - 1; // )0;
         }

         setYoVariableValuesToDataAtIndex();

         notifyIndexChangedListeners();

         // @todo: JEP 100514: Note that notifying the simulationRewoundListeners will happen in the GUI thread, not the simulation/control thread.
         // So there may be thread timing issues here. We may need to do some sort of synchronization and/or change it so that the
         // simulationRewoundListeners are notified in the simulation/control thread.
         if (notifySimulationRewoundListeners)
            notifyRewindListeners();
      }
   }

   public void attachSimulationRewoundListeners(List<RewoundListener> simulationRewoundListeners)
   {
      for (RewoundListener simulationRewoundListener : simulationRewoundListeners)
      {
         attachSimulationRewoundListener(simulationRewoundListener);
      }
   }

   public void attachSimulationRewoundListener(RewoundListener simulationRewoundListener)
   {
      if (simulationRewoundListeners == null)
      {
         simulationRewoundListeners = new ArrayList<>();
      }

      simulationRewoundListeners.add(simulationRewoundListener);
   }

   public void attachIndexChangedListener(IndexChangedListener indexChangedListener)
   {
      if (indexChangedListeners == null)
      {
         indexChangedListeners = new ArrayList<>();
      }

      indexChangedListeners.add(indexChangedListener);
   }

   public void detachIndexChangedListener(IndexChangedListener indexChangedListener)
   {
      if (indexChangedListeners != null)
      {
         indexChangedListeners.add(indexChangedListener);
      }
   }

   @Override
   public int getIndex()
   {
      return index;
   }

   @Override
   public boolean tick(int ticks)
   {
      return tick(ticks, true);
   }

   @Override
   public boolean tickButDoNotNotifySimulationRewoundListeners(int ticks)
   {
      return tick(ticks, false);
   }

   /**
    * This method attempts to step the index n points. If the offset is within the valid data set the
    * function returns false and the index is set to index+n. Otherwise the index is forced to the
    * inPoint or the outPoint depending on which is more appropriate.
    *
    * @param n Number of steps to shift the index, this value can be negative.
    * @return Indicates whether or not the index was forced to one of the ends.
    */
   private boolean tick(int n, boolean notifySimulationRewoundListeners)
   {
      if (safeToManuallyChangeIndex)
      {
         int newIndex = index + n;

         boolean rolledOver = !isIndexBetweenInAndOutPoint(newIndex);

         if (rolledOver)
         {
            if (n >= 0)
            {
               newIndex = inPoint;
            }
            else
            {
               newIndex = outPoint;
            }
         }

         setIndex(newIndex, notifySimulationRewoundListeners);

         return rolledOver;
      }

      return false;
   }

   public boolean updateAndTick()
   {
      setDataAtIndexToYoVariableValues();
      boolean ret = tick(1);
      setYoVariableValuesToDataAtIndex();

      return ret;
   }

   public boolean updateAndTickBackwards()
   {
      setDataAtIndexToYoVariableValues();
      boolean ret = tick(-1);
      setYoVariableValuesToDataAtIndex();

      return ret;
   }

   private void setYoVariableValuesToDataAtIndex()
   {
      //noinspection ForLoopReplaceableByForEach (iterators use memory, runs in tight loop)
      for (int j = 0; j < entries.size(); j++)
      {
         DataBufferEntry entry = entries.get(j);
         entry.setYoVariableValueToDataAtIndex(index);
      }
   }

   public void setDataAtIndexToYoVariableValues()
   {
      //noinspection ForLoopReplaceableByForEach (iterators use memory, runs in tight loop)
      for (int j = 0; j < entries.size(); j++)
      {
         DataBufferEntry entry = entries.get(j);
         entry.setDataAtIndexToYoVariableValue(index);
      }
   }

   public void tickAndUpdate()
   {
      if (!clearing)
      {
         index = index + 1;

         if (index >= bufferSize)
         {
            if (wrapBuffer || bufferSize >= maxBufferSize)
            {
               index = 0;
            }
            else // Expand the buffer, it just overflowed and there's room to grow...
            {
               int newSize = bufferSize * 3 / 2;

               if (newSize > maxBufferSize)
               {
                  newSize = maxBufferSize;
               }

               enlargeBufferSize(newSize);
            }
         }

         if (index < 0)
         {
            index = 0;
         }

         // Out point should always be the last recorded tick...
         outPoint = index;

         if (outPoint == inPoint)
         {
            inPoint = inPoint + 1;

            if (inPoint >= bufferSize)
            {
               inPoint = 0;
            }
         }

         keyPoints.removeKeyPoint(index);
         setDataAtIndexToYoVariableValues();
         notifyIndexChangedListeners();
      }
      else
      {
         clearing = false;
         resetDataBuffer();
      }
   }

   public void notifyRewindListeners()
   {
      if (simulationRewoundListeners != null)
      {
         for (int i = 0; i < simulationRewoundListeners.size(); i++)
         {
            RewoundListener simulationRewoundListener = simulationRewoundListeners.get(i);

            simulationRewoundListener.notifyOfRewind();
         }
      }

      notifyDataBufferListeners();
   }

   private void notifyIndexChangedListeners()
   {
      if (indexChangedListeners != null)
      {
         for (int i = 0; i < indexChangedListeners.size(); i++)
         {
            indexChangedListeners.get(i).notifyOfIndexChange(index);
         }
      }

      notifyDataBufferListeners();
   }

   private void notifyDataBufferListeners()
   {
      for (int i = 0; i < dataBufferListeners.size(); i++)
      {
         DataBufferListener dataBufferListener = dataBufferListeners.get(i);
         YoVariable[] yoVariables = dataBufferListener.getVariablesOfInterest(yoVariableList);
         double[] values = new double[yoVariables.length];

         for (int j = 0; j < yoVariables.length; j++)
         {
            values[j] = yoVariables[j].getValueAsDouble();
         }

         dataBufferListener.dataBufferUpdate(values);
      }
   }

   public void applyDataProcessingFunction(DataProcessingFunction dataProcessingFunction)
   {
      dataProcessingFunction.initializeProcessing();
      gotoInPoint();

      while (!atOutPoint())
      {
         dataProcessingFunction.processData();
         updateAndTick();
      }

      dataProcessingFunction.processData();
      updateAndTick();
   }

   public void applyDataProcessingFunctionBackward(DataProcessingFunction dataProcessingFunction)
   {
      dataProcessingFunction.initializeProcessing();
      gotoOutPoint();

      while (!atInPoint())
      {
         dataProcessingFunction.processData();
         updateAndTickBackwards();
      }

      dataProcessingFunction.processData();
      updateAndTickBackwards();
   }

   @Override
   public boolean isKeyPointModeToggled()
   {
      return keyPoints.useKeyPoints();
   }

   @Override
   public void toggleKeyPointMode()
   {
      if (keyPoints.useKeyPoints())
      {
         keyPoints.setUseKeyPoints(false);
      }
      else
      {
         keyPoints.setUseKeyPoints(true);
      }

      for (ToggleKeyPointModeCommandListener commandListener : toggleKeyPointModeCommandListeners)
      {
         commandListener.updateKeyPointModeStatus();
      }
   }

   @Override
   public void registerToggleKeyPointModeCommandListener(ToggleKeyPointModeCommandListener commandListener)
   {
      toggleKeyPointModeCommandListeners.add(commandListener);
   }

   public int getNextTime()
   {
      return keyPoints.getNextTime(index);
   }

   public int getPreviousTime()
   {
      return keyPoints.getPreviousTime(index);
   }

   public List<YoVariable> getVariablesThatStartWith(String searchString, boolean caseSensitive)
   {
      ArrayList<YoVariable> ret = null;

      if (!caseSensitive)
      {
         searchString = searchString.toLowerCase();
      }

      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);
         String name = entry.getVariable().getName();

         if (!caseSensitive)
         {
            name = name.toLowerCase();
         }

         if (name.startsWith(searchString))
         {
            if (ret == null)
            {
               ret = new ArrayList<>();
            }

            ret.add(entry.getVariable());
         }
      }

      return ret;
   }

   public boolean checkIfDataIsEqual(DataBuffer dataBuffer, double epsilon)
   {
      ArrayList<DataBufferEntry> thisEntries = entries;
      ArrayList<DataBufferEntry> entries = dataBuffer.entries;

      if (thisEntries.size() != entries.size())
      {
         System.out.println("Sizes don't match! thisEntries.size() = " + thisEntries.size() + ", entries.size() = " + entries.size());

         return false;
      }

      for (DataBufferEntry entry : entries)
      {
         YoVariable variable = entry.getVariable();
         DataBufferEntry entry2 = this.getEntry(variable.getName());

         if (entry2 == null)
         {
            System.out.println("Dont' have the same variables! Can't find " + variable.getName());

            return false;
         }

         if (!entry.checkIfDataIsEqual(entry2, inPoint, outPoint, epsilon))
         {
            System.out.println("Data in entries are different!");

            return false;
         }
      }

      return true;
   }

   public String getTimeVariableName()
   {
      return timeVariableName;
   }

   public void setTimeVariableName(String timeVariableName)
   {
      if (getEntry(timeVariableName) == null)
      {
         System.err.println("The requested timeVariableName does not exist, change not successful");
      }
      else
      {
         this.timeVariableName = timeVariableName;
      }
   }

   @Override
   public double[] getTimeData()
   {
      return getEntry(timeVariableName).getData();
   }

   @Override
   public boolean isIndexBetweenInAndOutPoint(int indexToCheck)
   {
      if (inPoint <= outPoint)
      {
         if (indexToCheck >= inPoint && indexToCheck <= outPoint)
         {
            return true;
         }
      }
      else
      {
         if (indexToCheck <= outPoint || indexToCheck > inPoint)
         {
            return true;
         }
      }

      return false;
   }

   public YoVariableList getYoVariableList()
   {
      return yoVariableList;
   }
}
