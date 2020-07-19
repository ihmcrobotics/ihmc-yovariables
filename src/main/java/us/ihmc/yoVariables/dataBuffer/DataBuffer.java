package us.ihmc.yoVariables.dataBuffer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class DataBuffer implements YoVariableHolder, DataBufferCommandsExecutor, TimeDataHolder, DataEntryHolder
{
   private String timeVariableName = "t";

   private int inPoint = 0;
   private int outPoint = 0;
   private int index = 0;
   private int bufferSize;
   private int maxBufferSize = 16384;

   private boolean wrapBuffer = false; // Default to Expand, not Wrap!  true;

   private final ArrayList<DataBufferEntry> entries = new ArrayList<>();
   private final HashMap<String, List<DataBufferEntry>> simpleNameToEntriesMap = new HashMap<>();

   private final KeyPointsHandler keyPointsHandler = new KeyPointsHandler();
   private final List<IndexChangedListener> indexChangedListeners = new ArrayList<>();

   private boolean lockIndex = false;

   public DataBuffer(int bufferSize)
   {
      this.bufferSize = bufferSize;
   }

   public DataBuffer(DataBuffer other)
   {
      inPoint = other.inPoint;
      outPoint = other.outPoint;
      index = other.index;
      bufferSize = other.bufferSize;
      maxBufferSize = other.maxBufferSize;
      wrapBuffer = other.wrapBuffer;
      lockIndex = other.lockIndex;

      for (DataBufferEntry otherEntry : other.entries)
         addEntry(new DataBufferEntry(otherEntry));
   }

   public void clear()
   {
      entries.clear();
      index = -1;
   }

   public void setLockIndex(boolean lock)
   {
      lockIndex = lock;
   }

   public boolean isIndexLocked()
   {
      return lockIndex;
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
         throw new IllegalArgumentException("The new entry size (" + entry.getDataLength() + ") does not match the buffer size (" + bufferSize + ").");

      entries.add(entry);

      String variableName = entry.getVariable().getName().toLowerCase();
      List<DataBufferEntry> entryList = simpleNameToEntriesMap.get(variableName);
      if (entryList == null)
      {
         entryList = new ArrayList<>();
         simpleNameToEntriesMap.put(variableName, entryList);
      }
      entryList.add(entry);
   }

   public DataBufferEntry addVariable(YoVariable variable)
   {
      DataBufferEntry entry = new DataBufferEntry(variable, bufferSize);
      addEntry(entry);
      return entry;
   }

   public void addVariables(List<? extends YoVariable> variables)
   {
      // do this first so that 'entries' will only have to grow once.
      entries.ensureCapacity(entries.size() + variables.size());

      for (int i = 0; i < variables.size(); i++)
      {
         this.addVariable(variables.get(i));
      }
   }

   @Override
   public DataBufferEntry getEntry(YoVariable variable)
   {
      for (DataBufferEntry entry : entries)
      {
         if (entry.getVariable() == variable)
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

   @Override
   public List<YoVariable> getVariables()
   {
      return entries.stream().map(DataBufferEntry::getVariable).collect(Collectors.toList());
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

   public void clearAll(int bufferSize)
   {
      entries.forEach(entry -> entry.clear(bufferSize));
      this.bufferSize = bufferSize;
   }

   public void resizeBuffer(int newBufferSize)
   {
      if (newBufferSize < bufferSize)
      {
         cropData(inPoint, (inPoint + newBufferSize - 1) % bufferSize);
      }
      else if (newBufferSize > bufferSize)
      {
         packData();
         enlargeBufferSize(newBufferSize);
      }
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
      keyPointsHandler.trimKeyPoints(inPoint, outPoint);
   }

   public void setOutPoint(int out)
   {
      outPoint = out;
      keyPointsHandler.trimKeyPoints(inPoint, outPoint);
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
      keyPointsHandler.toggleKeyPoint(index);
   }

   @Override
   public void setIndex(int index)
   {
      if (lockIndex)
         return;

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
   }

   public void attachIndexChangedListener(IndexChangedListener indexChangedListener)
   {
      indexChangedListeners.add(indexChangedListener);
   }

   public boolean detachIndexChangedListener(IndexChangedListener indexChangedListener)
   {
      return indexChangedListeners.remove(indexChangedListener);
   }

   @Override
   public int getIndex()
   {
      return index;
   }

   /**
    * This method attempts to step the index n points. If the offset is within the valid data set the
    * function returns false and the index is set to index+n. Otherwise the index is forced to the
    * inPoint or the outPoint depending on which is more appropriate.
    *
    * @param n Number of steps to shift the index, this value can be negative.
    * @return Indicates whether or not the index was forced to one of the ends.
    */
   @Override
   public boolean tick(int ticks)
   {
      if (lockIndex)
         return false;

      int newIndex = index + ticks;

      boolean rolledOver = !isIndexBetweenInAndOutPoint(newIndex);

      if (rolledOver)
      {
         if (ticks >= 0)
         {
            newIndex = inPoint;
         }
         else
         {
            newIndex = outPoint;
         }
      }

      setIndex(newIndex);

      return rolledOver;
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

      keyPointsHandler.removeKeyPoint(index);
      setDataAtIndexToYoVariableValues();
      notifyIndexChangedListeners();
   }

   private void notifyIndexChangedListeners()
   {
      for (int i = 0; i < indexChangedListeners.size(); i++)
      {
         indexChangedListeners.get(i).notifyOfIndexChange(index);
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
         DataBufferEntry entry2 = findVariableEntry(variable.getName());

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
      if (findVariableEntry(timeVariableName) == null)
      {
         System.err.println("The requested timeVariableName does not exist, change not successful");
      }
      else
      {
         this.timeVariableName = timeVariableName;
      }
   }

   public KeyPointsHandler getKeyPointsHandler()
   {
      return keyPointsHandler;
   }

   public int getNextKeyPoint()
   {
      return keyPointsHandler.getNextKeyPoint(index);
   }

   public int getPreviousKeyPoint()
   {
      return keyPointsHandler.getPreviousKeyPoint(index);
   }

   @Override
   public double[] getTimeData()
   {
      return findVariableEntry(timeVariableName).getBuffer();
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

   @Override
   public YoVariable findVariable(String nameSpaceEnding, String name)
   {
      DataBufferEntry entry = findVariableEntry(nameSpaceEnding, name);
      return entry == null ? null : entry.getVariable();
   }

   public DataBufferEntry findVariableEntry(String name)
   {
      int separatorIndex = name.lastIndexOf(YoTools.NAMESPACE_SEPERATOR_STRING);

      if (separatorIndex == -1)
         return findVariableEntry(null, name);
      else
         return findVariableEntry(name.substring(0, separatorIndex), name.substring(separatorIndex + 1));
   }

   public DataBufferEntry findVariableEntry(String nameSpaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      List<DataBufferEntry> entryList = simpleNameToEntriesMap.get(name.toLowerCase());

      if (entryList == null || entryList.isEmpty())
         return null;

      if (nameSpaceEnding == null)
      {
         return entryList.get(0);
      }
      else
      {
         for (int i = 0; i < entryList.size(); i++)
         {
            DataBufferEntry candidate = entryList.get(i);

            if (candidate.getVariable().getNameSpace().endsWith(nameSpaceEnding, true))
               return candidate;
         }
      }

      return null;
   }

   @Override
   public List<YoVariable> findVariables(String nameSpaceEnding, String name)
   {
      return findVariableEntries(nameSpaceEnding, name).stream().map(DataBufferEntry::getVariable).collect(Collectors.toList());
   }

   public List<DataBufferEntry> findVariableEntries(String nameSpaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      List<DataBufferEntry> entryList = simpleNameToEntriesMap.get(name.toLowerCase());

      if (entryList == null || entryList.isEmpty())
         return Collections.emptyList();

      List<DataBufferEntry> result = new ArrayList<>();

      if (nameSpaceEnding == null)
      {
         result.addAll(entryList);
      }
      else
      {
         for (int i = 0; i < entryList.size(); i++)
         {
            DataBufferEntry candidate = entryList.get(i);

            if (candidate.getVariable().getNameSpace().endsWith(nameSpaceEnding, true))
               result.add(candidate);
         }
      }

      return result;
   }

   @Override
   public List<YoVariable> findVariables(NameSpace nameSpace)
   {
      return findVariableEntries(nameSpace).stream().map(DataBufferEntry::getVariable).collect(Collectors.toList());
   }

   public List<DataBufferEntry> findVariableEntries(NameSpace nameSpace)
   {
      List<DataBufferEntry> result = new ArrayList<>();

      for (DataBufferEntry entry : entries)
      {
         if (entry.getVariable().getNameSpace().equals(nameSpace))
         {
            result.add(entry);
         }
      }

      return result;
   }

   @Override
   public List<YoVariable> filterVariables(Predicate<YoVariable> filter)
   {
      return filterVariableEntries(filter).stream().map(DataBufferEntry::getVariable).collect(Collectors.toList());
   }

   public List<DataBufferEntry> filterVariableEntries(Predicate<YoVariable> filter)
   {
      List<DataBufferEntry> result = new ArrayList<>();

      for (DataBufferEntry entry : entries)
      {
         if (filter.test(entry.getVariable()))
            result.add(entry);
      }
      return result;
   }

   @Override
   public boolean hasUniqueVariable(String nameSpaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      return countNumberOfEntries(nameSpaceEnding, name) == 1;
   }

   private int countNumberOfEntries(String parentNameSpace, String name)
   {
      List<DataBufferEntry> entryList = simpleNameToEntriesMap.get(name.toLowerCase());

      if (entryList == null || entryList.isEmpty())
         return 0;

      if (parentNameSpace == null)
         return entryList.size();

      int count = 0;

      for (int i = 0; i < entryList.size(); i++)
      {
         if (entryList.get(i).getVariable().getNameSpace().endsWith(parentNameSpace, true))
            count++;
      }
      return count;
   }
}
