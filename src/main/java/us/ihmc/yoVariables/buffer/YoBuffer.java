package us.ihmc.yoVariables.buffer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferIndexChangedListener;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferProcessor;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferReader;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferVariableEntryHolder;
import us.ihmc.yoVariables.buffer.interfaces.YoTimeBufferHolder;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoBuffer implements YoVariableHolder, YoBufferReader, YoTimeBufferHolder, YoBufferVariableEntryHolder
{
   private String timeVariableName = "t";

   private int inPoint = 0;
   private int outPoint = 0;
   private int currentIndex = 0;
   private int bufferSize;

   private final ArrayList<YoBufferVariableEntry> entries = new ArrayList<>();
   private final HashMap<String, List<YoBufferVariableEntry>> simpleNameToEntriesMap = new HashMap<>();

   private final KeyPointsHandler keyPointsHandler = new KeyPointsHandler();
   private final List<YoBufferIndexChangedListener> indexChangedListeners = new ArrayList<>();

   private boolean lockIndex = false;

   public YoBuffer(int bufferSize)
   {
      this.bufferSize = bufferSize;
   }

   public YoBuffer(YoBuffer other)
   {
      inPoint = other.inPoint;
      outPoint = other.outPoint;
      currentIndex = other.currentIndex;
      bufferSize = other.bufferSize;
      lockIndex = other.lockIndex;

      for (YoBufferVariableEntry otherEntry : other.entries)
         addEntry(new YoBufferVariableEntry(otherEntry));
   }

   public void clear()
   {
      entries.clear();
      currentIndex = -1;
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

   public void addEntry(YoBufferVariableEntry entry)
   {
      if (entry.getBufferSize() != bufferSize)
         throw new IllegalArgumentException("The new entry size (" + entry.getBufferSize() + ") does not match the buffer size (" + bufferSize + ").");

      entries.add(entry);

      String variableName = entry.getVariable().getName().toLowerCase();
      List<YoBufferVariableEntry> entryList = simpleNameToEntriesMap.get(variableName);
      if (entryList == null)
      {
         entryList = new ArrayList<>();
         simpleNameToEntriesMap.put(variableName, entryList);
      }
      entryList.add(entry);
   }

   public YoBufferVariableEntry addVariable(YoVariable variable)
   {
      YoBufferVariableEntry entry = new YoBufferVariableEntry(variable, bufferSize);
      addEntry(entry);
      return entry;
   }

   public void addVariables(List<? extends YoVariable> variables)
   {
      // do this first so that 'entries' will only have to grow once.
      entries.ensureCapacity(entries.size() + variables.size());

      for (int i = 0; i < variables.size(); i++)
      {
         addVariable(variables.get(i));
      }
   }

   @Override
   public YoBufferVariableEntry getEntry(YoVariable variable)
   {
      for (YoBufferVariableEntry entry : entries)
      {
         if (entry.getVariable() == variable)
         {
            return entry;
         }
      }

      return null;
   }

   public List<YoBufferVariableEntry> getEntries()
   {
      return entries;
   }

   @Override
   public List<YoVariable> getVariables()
   {
      return entries.stream().map(YoBufferVariableEntry::getVariable).collect(Collectors.toList());
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
         YoBufferVariableEntry entry = entries.get(i);
         entry.enlargeBufferSize(newSize);
      }

      bufferSize = newSize;
   }

   public void copyValuesThrough()
   {
      for (int i = 0; i < entries.size(); i++)
      {
         YoBufferVariableEntry entry = entries.get(i);

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
         YoBufferVariableEntry entry = entries.get(i);

         entry.packData(start);
      }

      // Move the current index to its relative position in the new data set, if the index is outside of the buffer move to zero
      currentIndex = (currentIndex - start + bufferSize) % bufferSize;

      if (currentIndex < 0)
      {
         currentIndex = 0;
      }

      // Move the inPoint to the new beginning and the outPoint to the end
      inPoint = 0; // this.inPoint - start;
      outPoint = (outPoint - start + bufferSize) % bufferSize;

      // Move to the first tick
      tickAndReadFromBuffer(0);

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
         bufferSize = YoBufferVariableEntry.computeBufferSizeAfterCrop(start, end, bufferSize);
      }

      // Step through the entries cropping and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         YoBufferVariableEntry entry = entries.get(i);
         int retSize = entry.cropData(start, end);

         // If the result is a positive number store the new buffer size otherwise keep the original size
         if (retSize >= 0)
         {
            bufferSize = retSize;
         }
      }

      // Move the current index to its relative position after the resize
      currentIndex = (currentIndex - start + bufferSize) % bufferSize;

      // If the index is out of bounds move it to the beginning
      if (currentIndex < 0 || currentIndex >= bufferSize)
      {
         currentIndex = 0;
      }

      // Set the in point to the beginning and the out point to the end
      inPoint = 0;
      outPoint = bufferSize - 1;

      // Move to the first tick
      tickAndReadFromBuffer(0);
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
         bufferSize = YoBufferVariableEntry.computeBufferSizeAfterCut(start, end, bufferSize);
      }

      // Step through the entries cutting and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         YoBufferVariableEntry entry = entries.get(i);
         int retSize = entry.cutData(start, end);

         // If the result is a positive number store the new buffer size otherwise keep the original size
         if (retSize >= 0)
         {
            bufferSize = retSize;
         }
      }

      // Move the current index to its relative position after the resize
      currentIndex = (currentIndex - start + bufferSize) % bufferSize;

      // If the index is out of bounds move it to the beginning
      if (currentIndex < 0 || currentIndex >= bufferSize)
      {
         currentIndex = 0;
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
      currentIndex = 0;

      if (bufferSize <= 2 * keepEveryNthPoint)
         return;

      // Step through the entries cutting and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         YoBufferVariableEntry entry = entries.get(i);
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
      YoBufferVariableEntry entry = getEntry(variable);
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
      setInPoint(currentIndex);
   }

   public void setOutPoint()
   {
      setOutPoint(currentIndex);
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
      outPoint = entries.get(0).getBufferSize() - 1;
   }

   public void gotoInPoint()
   {
      setCurrentIndex(inPoint);
   }

   public void gotoOutPoint()
   {
      setCurrentIndex(outPoint);
   }

   public boolean atInPoint()
   {
      return currentIndex == inPoint;
   }

   public boolean atOutPoint()
   {
      return currentIndex == outPoint;
   }

   public void setKeyPoint()
   {
      keyPointsHandler.toggleKeyPoint(currentIndex);
   }

   public void readFromBuffer()
   {
      for (int i = 0; i < entries.size(); i++)
      {
         entries.get(i).readFromBufferAt(currentIndex);
      }
   }

   public void writeIntoBuffer()
   {
      for (int i = 0; i < entries.size(); i++)
      {
         entries.get(i).writeIntoBufferAt(currentIndex);
      }
   }

   @Override
   public void setCurrentIndex(int index)
   {
      if (lockIndex)
         return;

      currentIndex = index;

      if (currentIndex >= bufferSize)
         currentIndex = 0;
      else if (currentIndex < 0)
         currentIndex = bufferSize - 1;

      readFromBuffer();
      notifyIndexChangedListeners();
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
   public boolean tickAndReadFromBuffer(int stepSize)
   {
      if (lockIndex)
         return false;

      int newIndex = currentIndex + stepSize;

      boolean rolledOver = !isIndexBetweenBounds(newIndex);

      if (rolledOver)
      {
         newIndex = stepSize >= 0 ? inPoint : outPoint;
      }

      setCurrentIndex(newIndex);

      return rolledOver;
   }

   public void tickAndWriteIntoBuffer()
   {
      currentIndex = currentIndex + 1;

      if (currentIndex >= bufferSize || currentIndex < 0)
         currentIndex = 0;

      // Out point should always be the last recorded tick...
      outPoint = currentIndex;

      if (outPoint == inPoint)
      {
         inPoint++;

         if (inPoint >= bufferSize)
            inPoint = 0;
      }

      keyPointsHandler.removeKeyPoint(currentIndex);
      writeIntoBuffer();
      notifyIndexChangedListeners();
   }

   public void attachIndexChangedListener(YoBufferIndexChangedListener indexChangedListener)
   {
      indexChangedListeners.add(indexChangedListener);
   }

   public boolean detachIndexChangedListener(YoBufferIndexChangedListener indexChangedListener)
   {
      return indexChangedListeners.remove(indexChangedListener);
   }

   @Override
   public int getCurrentIndex()
   {
      return currentIndex;
   }

   private void notifyIndexChangedListeners()
   {
      for (int i = 0; i < indexChangedListeners.size(); i++)
      {
         indexChangedListeners.get(i).notifyOfIndexChange(currentIndex);
      }
   }

   public void applyProcessor(YoBufferProcessor processor)
   {
      processor.initialize(this);

      if (processor.goForward())
      {
         gotoInPoint();

         while (!atOutPoint())
         {
            processor.process(inPoint, outPoint, currentIndex);
            writeIntoBuffer();
            tickAndReadFromBuffer(1);
         }
         
         processor.process(inPoint, outPoint, currentIndex);
         writeIntoBuffer();
         tickAndReadFromBuffer(1);
      }
      else
      {
         gotoOutPoint();
         
         while (!atInPoint())
         {
            processor.process(outPoint, inPoint, currentIndex);
            writeIntoBuffer();
            tickAndReadFromBuffer(-1);
         }
         
         processor.process(outPoint, inPoint, currentIndex);
         writeIntoBuffer();
         tickAndReadFromBuffer(-1);
      }
   }

   public boolean checkIfDataIsEqual(YoBuffer dataBuffer, double epsilon)
   {
      ArrayList<YoBufferVariableEntry> thisEntries = entries;
      ArrayList<YoBufferVariableEntry> entries = dataBuffer.entries;

      if (thisEntries.size() != entries.size())
      {
         System.out.println("Sizes don't match! thisEntries.size() = " + thisEntries.size() + ", entries.size() = " + entries.size());

         return false;
      }

      for (YoBufferVariableEntry entry : entries)
      {
         YoVariable variable = entry.getVariable();
         YoBufferVariableEntry entry2 = findVariableEntry(variable.getName());

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
         LogTools.error("The requested timeVariableName does not exist, change not successful");
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
      return keyPointsHandler.getNextKeyPoint(currentIndex);
   }

   public int getPreviousKeyPoint()
   {
      return keyPointsHandler.getPreviousKeyPoint(currentIndex);
   }

   @Override
   public double[] getTimeBuffer()
   {
      return findVariableEntry(timeVariableName).getBuffer();
   }

   @Override
   public YoVariable findVariable(String nameSpaceEnding, String name)
   {
      YoBufferVariableEntry entry = findVariableEntry(nameSpaceEnding, name);
      return entry == null ? null : entry.getVariable();
   }

   public YoBufferVariableEntry findVariableEntry(String name)
   {
      int separatorIndex = name.lastIndexOf(YoTools.NAMESPACE_SEPERATOR_STRING);

      if (separatorIndex == -1)
         return findVariableEntry(null, name);
      else
         return findVariableEntry(name.substring(0, separatorIndex), name.substring(separatorIndex + 1));
   }

   public YoBufferVariableEntry findVariableEntry(String nameSpaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      List<YoBufferVariableEntry> entryList = simpleNameToEntriesMap.get(name.toLowerCase());

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
            YoBufferVariableEntry candidate = entryList.get(i);

            if (candidate.getVariable().getNameSpace().endsWith(nameSpaceEnding, true))
               return candidate;
         }
      }

      return null;
   }

   @Override
   public List<YoVariable> findVariables(String nameSpaceEnding, String name)
   {
      return findVariableEntries(nameSpaceEnding, name).stream().map(YoBufferVariableEntry::getVariable).collect(Collectors.toList());
   }

   public List<YoBufferVariableEntry> findVariableEntries(String nameSpaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      List<YoBufferVariableEntry> entryList = simpleNameToEntriesMap.get(name.toLowerCase());

      if (entryList == null || entryList.isEmpty())
         return Collections.emptyList();

      List<YoBufferVariableEntry> result = new ArrayList<>();

      if (nameSpaceEnding == null)
      {
         result.addAll(entryList);
      }
      else
      {
         for (int i = 0; i < entryList.size(); i++)
         {
            YoBufferVariableEntry candidate = entryList.get(i);

            if (candidate.getVariable().getNameSpace().endsWith(nameSpaceEnding, true))
               result.add(candidate);
         }
      }

      return result;
   }

   @Override
   public List<YoVariable> findVariables(NameSpace nameSpace)
   {
      return findVariableEntries(nameSpace).stream().map(YoBufferVariableEntry::getVariable).collect(Collectors.toList());
   }

   public List<YoBufferVariableEntry> findVariableEntries(NameSpace nameSpace)
   {
      List<YoBufferVariableEntry> result = new ArrayList<>();

      for (YoBufferVariableEntry entry : entries)
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
      return filterVariableEntries(filter).stream().map(YoBufferVariableEntry::getVariable).collect(Collectors.toList());
   }

   public List<YoBufferVariableEntry> filterVariableEntries(Predicate<YoVariable> filter)
   {
      List<YoBufferVariableEntry> result = new ArrayList<>();

      for (YoBufferVariableEntry entry : entries)
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
      List<YoBufferVariableEntry> entryList = simpleNameToEntriesMap.get(name.toLowerCase());

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
