package us.ihmc.yoVariables.buffer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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

/**
 * {@code YoBuffer} manages buffers to store history for a collection of {@link YoVariable}s.
 * <p>
 * When adding a variable to this buffer via {@link #addVariable(YoVariable)}, a new
 * {@link YoBufferVariableEntry} for that new variable is created.
 * </p>
 * <p>
 * Once variables have been registered, the two main methods of interest are:
 * <ul>
 * <li>{@link #tickAndWriteIntoBuffer()} to take a single step in the buffer, then writes the
 * {@code YoVariable} values into the buffer.
 * <li>{@link #tickAndReadFromBuffer(int)} to step forward or backward in the buffer, then reads the
 * buffer and updates the {@code YoVariable}s.
 * <li>{@link #setCurrentIndex(int)} to change the current position in the buffer, then reads the
 * buffer and updates the {@code YoVariable}s.
 * </ul>
 * </p>
 */
public class YoBuffer implements YoVariableHolder, YoBufferReader, YoTimeBufferHolder, YoBufferVariableEntryHolder
{
   /** Name for the the time variable. */
   private String timeVariableName = "t";

   /**
    * The index of the current buffer's in-point.
    * <p>
    * The buffer has a sub-interval to highlight part of interest in the buffer, or the part were
    * actual data was written. This sub-interval is defined by [{@code inPoint}, {@code outPoint}].
    * When filled up, the buffer automatically wraps the reading/writing index, such that it is
    * possible that {@code inPoint > outPoint} indicating that the sub-interval starts towards the end
    * of the buffer to end at the beginning.
    * </p>
    */
   private int inPoint = 0;
   /**
    * The index of the current buffer's out-point.
    * <p>
    * The buffer has a sub-interval to highlight part of interest in the buffer, or the part were
    * actual data was written. This sub-interval is defined by [{@code inPoint}, {@code outPoint}].
    * When filled up, the buffer automatically wraps the reading/writing index, such that it is
    * possible that {@code inPoint > outPoint} indicating that the sub-interval starts towards the end
    * of the buffer to end at the beginning.
    * </p>
    */
   private int outPoint = 0;
   /**
    * The current index represents the current reading/writing position in the buffer.
    */
   private int currentIndex = 0;
   /** The current buffer size. */
   private int bufferSize;
   /** List of all the single variable buffers. */
   private final ArrayList<YoBufferVariableEntry> entries = new ArrayList<>();
   /**
    * Mapping from variable name (lower case) to buffer entries to facilitates entry and variable
    * retrieval.
    */
   private final Map<String, List<YoBufferVariableEntry>> simpleNameToEntriesMap = new HashMap<>();
   /**
    * Manages user defined key points used to highlight and keep track of key indices in the buffer.
    */
   private final KeyPointsHandler keyPointsHandler = new KeyPointsHandler();
   /** The list of listeners to be notified of changes on this buffer's current index. */
   private final List<YoBufferIndexChangedListener> indexChangedListeners = new ArrayList<>();

   private boolean lockIndex = false;

   /**
    * Creates a new empty buffer.
    * <p>
    * Variables can be registered to this buffer {@link #addVariable(YoVariable)}.
    * </p>
    *
    * @param bufferSize the initialize buffer size.
    */
   public YoBuffer(int bufferSize)
   {
      this.bufferSize = bufferSize;
   }

   /**
    * Clone constructor.
    *
    * @param other the other buffer to copy. Not modified.
    */
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

   /**
    * Clears the internal data and removed all variables and their buffers.
    */
   public void clear()
   {
      inPoint = 0;
      outPoint = 0;
      currentIndex = 0;
      entries.clear();
      simpleNameToEntriesMap.clear();
      keyPointsHandler.clear();
      indexChangedListeners.clear();
   }

   /**
    * Clears the internal buffers, i.e. overwrite the previously written data, and resize this buffer.
    * 
    * @param bufferSize the new buffer size.
    */
   public void clearBuffers(int bufferSize)
   {
      entries.forEach(entry -> entry.clearBuffer(bufferSize));
      this.bufferSize = bufferSize;
   }

   /**
    * Resizes this buffer.
    * <p>
    * The data in [{@code inPoint}, {@code outPoint}] is preserved when possible is shifted to the
    * beginning of the buffer.
    * </p>
    * <p>
    * If {@code newBufferSize} is too small to retain the data in [{@code inPoint}, {@code outPoint}],
    * then the buffer is cropped to [{@code inPoint}, {@code inPoint + newBufferSize - 1}].
    * </p>
    * 
    * @param newBufferSize the new size for this buffer.
    */
   public void resizeBuffer(int newBufferSize)
   {
      if (newBufferSize < bufferSize)
      {
         cropBuffer(inPoint, (inPoint + newBufferSize - 1) % bufferSize);
      }
      else if (newBufferSize > bufferSize)
      {
         shiftBuffer();
         enlargeBufferSize(newBufferSize);
      }
   }

   private void enlargeBufferSize(int newBufferSize)
   {
      for (int i = 0; i < entries.size(); i++)
      {
         YoBufferVariableEntry entry = entries.get(i);
         entry.enlargeBufferSize(newBufferSize);
      }

      bufferSize = newBufferSize;
   }

   /**
    * Sets whether to lock the current index of this buffer.
    * <p>
    * The lock affects {@link #setCurrentIndex(int)}, {@link #tickAndReadFromBuffer(int)}, and
    * {@link #tickAndWriteIntoBuffer()}.
    * </p>
    *
    * @param lock {@code true} for preventing further changes to the reading position, {@code false} to
    *             unlock the current index.
    */
   public void setLockIndex(boolean lock)
   {
      lockIndex = lock;
   }

   /**
    * Returns whether this buffer's index is currently locked or not.
    *
    * @return {@code true} if the current index is locked and cannot be modified, {@code false}
    *         otherwise.
    */
   public boolean isIndexLocked()
   {
      return lockIndex;
   }

   /**
    * Adds the given entry to this buffer.
    * 
    * @param entry the new entry to be managed by this buffer.
    * @throws IllegalArgumentException if the given entry buffer size is different from this buffer
    *                                  size.
    */
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

   /**
    * Registers a variable to this buffer.
    * <p>
    * The new variable is being associated with a new buffer.
    * </p>
    * <p>
    * If the variable was already registered, this method does nothing and returns the existing
    * variable's entry.
    * </p>
    * 
    * @param variable the new variable to manage.
    * @return the newly created buffer for the new variable.
    */
   public YoBufferVariableEntry addVariable(YoVariable variable)
   {
      YoBufferVariableEntry entry = getEntry(variable);

      if (entry != null)
      {
         return entry;
      }
      else
      {
         entry = new YoBufferVariableEntry(variable, bufferSize);
         addEntry(entry);
         return entry;
      }
   }

   /**
    * Registers a list of variables.
    * <p>
    * The new variables are being associated with new individual buffers.
    * </p>
    * <p>
    * If a variable was already registered, it is skipped.
    * </p>
    * 
    * @param variables the list of variables to register to this buffer.
    */
   public void addVariables(List<? extends YoVariable> variables)
   {
      // do this first so that 'entries' will only have to grow once.
      entries.ensureCapacity(entries.size() + variables.size());

      for (int i = 0; i < variables.size(); i++)
      {
         addVariable(variables.get(i));
      }
   }

   /**
    * Removes a variable from this buffer.
    * 
    * @param variable the variable to remove from this buffer.
    * @return the variable associated buffer that was removed or {@code null} if it could not be found.
    */
   public YoBufferVariableEntry removeVariable(YoVariable variable)
   {
      YoBufferVariableEntry entry = getEntry(variable);

      if (entry == null)
         return null;

      entries.remove(entry);
      simpleNameToEntriesMap.get(variable.getName().toLowerCase()).remove(entry);
      return entry;
   }

   /**
    * Sets the current in-point for this buffer.
    * <p>
    * The buffer has a sub-interval to highlight part of interest in the buffer, or the part were
    * actual data was written. This sub-interval is defined by [{@code inPoint}, {@code outPoint}].
    * When filled up, the buffer automatically wraps the reading/writing index, such that it is
    * possible that {@code inPoint > outPoint} indicating that the sub-interval starts towards the end
    * of the buffer to end at the beginning.
    * </p>
    * 
    * @param index the new position of the in-point.
    */
   public void setInPoint(int index)
   {
      inPoint = index;
      keyPointsHandler.trimKeyPoints(inPoint, outPoint);
   }

   /**
    * Sets the current out-point for this buffer.
    * <p>
    * The buffer has a sub-interval to highlight part of interest in the buffer, or the part were
    * actual data was written. This sub-interval is defined by [{@code inPoint}, {@code outPoint}].
    * When filled up, the buffer automatically wraps the reading/writing index, such that it is
    * possible that {@code inPoint > outPoint} indicating that the sub-interval starts towards the end
    * of the buffer to end at the beginning.
    * </p>
    * 
    * @param index the new position of the out-point.
    */
   public void setOutPoint(int index)
   {
      outPoint = index;
      keyPointsHandler.trimKeyPoints(inPoint, outPoint);
   }

   /**
    * Sets the in-point to the current index.
    * 
    * @see #setInPoint(int)
    */
   public void setInPoint()
   {
      setInPoint(currentIndex);
   }

   /**
    * Sets the out-point to the current index.
    * 
    * @see #setOutPoint(int)
    */
   public void setOutPoint()
   {
      setOutPoint(currentIndex);
   }

   /**
    * Sets the in-point at {@code 0} and the out-point at the end of the buffer, i.e.
    * {@code getBufferSize() - 1}.
    */
   public void setInOutPointFullBuffer()
   {
      inPoint = 0;
      outPoint = getBufferSize() - 1;
   }

   /**
    * Tests if the current index is at the in-point.
    * 
    * @return {@code true} if the current index is at the in-point.
    */
   public boolean isAtInPoint()
   {
      return currentIndex == inPoint;
   }

   /**
    * Tests if the current index is at the out-point.
    * 
    * @return {@code true} if the current index is at the out-point.
    */
   public boolean isAtOutPoint()
   {
      return currentIndex == outPoint;
   }

   /**
    * Toggle a key point at the current index.
    * <p>
    * If no key point was present, it is created. If there was a key point, it is removed.
    * </p>
    */
   public void toggleKeyPoint()
   {
      keyPointsHandler.toggleKeyPoint(currentIndex);
   }

   /**
    * Reads the buffer at the current index and updates the values of the variables.
    */
   public void readFromBuffer()
   {
      for (int i = 0; i < entries.size(); i++)
      {
         entries.get(i).readFromBufferAt(currentIndex);
      }
   }

   /**
    * Write into the buffer at the current index the current values of the variables.
    */
   public void writeIntoBuffer()
   {
      for (int i = 0; i < entries.size(); i++)
      {
         entries.get(i).writeIntoBufferAt(currentIndex);
      }
   }

   /**
    * Sets the current index to the in-point and reads from the buffer.
    * 
    * @see #setCurrentIndex(int)
    * @see #readFromBuffer()
    */
   public void gotoInPoint()
   {
      setCurrentIndex(inPoint);
   }

   /**
    * Sets the current index to the out-point and reads from the buffer.
    * 
    * @see #setCurrentIndex(int)
    * @see #readFromBuffer()
    */
   public void gotoOutPoint()
   {
      setCurrentIndex(outPoint);
   }

   /** {@inheritDoc} */
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

   /** {@inheritDoc} */
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

   /**
    * Increments the current buffer index and write the values of the variables into the buffer at the
    * new index.
    */
   public void tickAndWriteIntoBuffer()
   {
      if (lockIndex)
         return;

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

   /**
    * Fills the buffer with the current values of the variables.
    */
   public void fillBuffer()
   {
      for (int i = 0; i < entries.size(); i++)
      {
         entries.get(i).fillBuffer();
      }
   }

   /**
    * Shifts the data in the buffer such that the in-point is at {@code 0}.
    */
   public void shiftBuffer()
   {
      shiftBuffer(inPoint);
   }

   /**
    * Shifts the data in the buffer such that the given index ends up at {@code 0}.
    * 
    * @param shiftIndex the shift is achieved such that the index ends at {@code 0} when the operation
    *                   is done. If the index is already at {@code 0}, nothing happens. If the index is
    *                   outside the buffer, nothing happens.
    */
   public void shiftBuffer(int shiftIndex)
   {
      if (shiftIndex == 0)
         return;

      // If the start point is outside of the buffer abort.
      if (shiftIndex <= 0 || shiftIndex >= bufferSize)
         return;

      // Shift the data in each entry to begin with start.
      for (int i = 0; i < entries.size(); i++)
         entries.get(i).shiftBuffer(shiftIndex);

      // Move the current index to its relative position in the new data set, if the index is outside of the buffer move to zero
      currentIndex = (currentIndex - shiftIndex + bufferSize) % bufferSize;

      if (currentIndex < 0)
         currentIndex = 0;

      // Move the inPoint to the new beginning and the outPoint to the end
      inPoint = 0; // this.inPoint - start;
      outPoint = (outPoint - shiftIndex + bufferSize) % bufferSize;

      // Move to the first tick
      tickAndReadFromBuffer(0);
   }

   /**
    * Crops the buffer to retain only the part that is in the interval [{@code inPoint},
    * {@code outPoint}].
    */
   public void cropBuffer()
   {
      if (inPoint != outPoint)
         cropBuffer(inPoint, outPoint);
      else
         cropBuffer(inPoint, inPoint + 1);
   }

   /**
    * Crops the buffer to retain only the part that is in the interval [{@code start}, {@code end}].
    * 
    * @param start the first index of the interval to retain.
    * @param end   the last index of the interval to retain.
    */
   public void cropBuffer(int start, int end)
   {
      // Abort if the start or end point is unreasonable
      if (start < 0 || end > bufferSize)
         return;

      bufferSize = YoBufferVariableEntry.computeBufferSizeAfterCrop(start, end, bufferSize);

      // Step through the entries cropping and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         entries.get(i).cropBuffer(start, end);
      }

      // Move the current index to its relative position after the resize
      currentIndex = (currentIndex - start + bufferSize) % bufferSize;

      // If the index is out of bounds move it to the beginning
      if (currentIndex < 0 || currentIndex >= bufferSize)
         currentIndex = 0;

      // Set the in point to the beginning and the out point to the end
      inPoint = 0;
      outPoint = bufferSize - 1;

      // Move to the first tick
      gotoInPoint();
   }

   /**
    * Cuts the buffer, removing the part that is in the interval [{@code inPoint}, {@code outPoint}].
    */
   public void cutBuffer()
   {
      if (inPoint <= outPoint)
      {
         cutBuffer(inPoint, outPoint);
      }
   }

   /**
    * Cuts the buffer, removing the part that is in the interval [{@code start}, {@code end}].
    * 
    * @param start the first index of the interval to remove.
    * @param end   the last index of the interval to remove.
    */
   public void cutBuffer(int start, int end)
   {
      // Abort if the start or end point is unreasonable
      if (start < 0 || end > bufferSize)
      {
         return;
      }

      bufferSize = YoBufferVariableEntry.computeBufferSizeAfterCut(start, end, bufferSize);

      // Step through the entries cutting and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         YoBufferVariableEntry entry = entries.get(i);
         entry.cutData(start, end);
      }

      // Move the current index to its relative position after the resize
      currentIndex = (currentIndex - start + bufferSize) % bufferSize;

      // If the index is out of bounds move it to the beginning
      if (currentIndex < 0 || currentIndex >= bufferSize)
         currentIndex = 0;

      // Set the in point to the beginning and the out point to the end
      inPoint = 0;
      outPoint = start - 1;

      // Move to the first tick
      gotoOutPoint();
   }

   /**
    * Prunes data and reduces buffer size by removing points and only keeping every {@code n} points.
    * The size of this buffer is essentially being divided by {@code n}.
    * 
    * @param n the spacing between data points to preserve.
    */
   public void thinData(int n)
   {
      shiftBuffer();

      inPoint = 0;
      currentIndex = 0;

      if (bufferSize <= 2 * n)
         return;

      // Step through the entries cutting and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         YoBufferVariableEntry entry = entries.get(i);
         int newBufferSize = entry.thinData(n);

         // If the result is a positive number store the new buffer size otherwise keep the original size
         if (newBufferSize >= 0)
         {
            bufferSize = newBufferSize;
         }
      }

      outPoint = bufferSize - 1;

      gotoInPoint();
   }

   /**
    * Computes and returns the average value for the given variable over its entire buffer.
    * 
    * @param variable the variable to compute the average of.
    * @return the variable's average.
    */
   public double computeAverage(YoVariable variable)
   {
      YoBufferVariableEntry entry = getEntry(variable);
      if (entry == null)
         return Double.NaN;
      else
         return entry.computeAverage();
   }

   /**
    * Applies a processor throughout the buffer to read and or modify this buffer.
    * 
    * @param processor the processor to applied to this buffer data.
    * @see YoBufferProcessor
    */
   public void applyProcessor(YoBufferProcessor processor)
   {
      processor.initialize(this);

      if (processor.goForward())
      {
         gotoInPoint();

         while (!isAtOutPoint())
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

         while (!isAtInPoint())
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

   /** {@inheritDoc} */
   @Override
   public int getInPoint()
   {
      return inPoint;
   }

   /** {@inheritDoc} */
   @Override
   public int getOutPoint()
   {
      return outPoint;
   }

   /** {@inheritDoc} */
   @Override
   public int getBufferSize()
   {
      return bufferSize;
   }

   /** {@inheritDoc} */
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

   /**
    * Returns all the variable entries that are managed by this buffer.
    * 
    * @return the buffer variable entries.
    */
   public List<YoBufferVariableEntry> getEntries()
   {
      return entries;
   }

   /** {@inheritDoc} */
   @Override
   public List<YoVariable> getVariables()
   {
      return entries.stream().map(YoBufferVariableEntry::getVariable).collect(Collectors.toList());
   }

   /**
    * Adds a listener to this buffer.
    *
    * @param listener the listener for listening to changes done to the current index.
    */
   public void addListener(YoBufferIndexChangedListener listener)
   {
      indexChangedListeners.add(listener);
   }

   /**
    * Removes all listeners previously added to this buffer.
    */
   public void removeListeners()
   {
      indexChangedListeners.clear();
   }

   /**
    * Tries to remove a listener from this buffer. If the listener could not be found and removed,
    * nothing happens.
    *
    * @param listener the listener to remove.
    * @return {@code true} if the listener was removed, {@code false} if the listener was not found and
    *         nothing happened.
    */
   public boolean removeListener(YoBufferIndexChangedListener listener)
   {
      return indexChangedListeners.remove(listener);
   }

   /** {@inheritDoc} */
   @Override
   public int getCurrentIndex()
   {
      return currentIndex;
   }

   private void notifyIndexChangedListeners()
   {
      for (int i = 0; i < indexChangedListeners.size(); i++)
      {
         indexChangedListeners.get(i).indexChanged(currentIndex);
      }
   }

   public boolean epsilonEquals(YoBuffer other, double epsilon)
   {
      List<YoBufferVariableEntry> thisEntries = entries;
      List<YoBufferVariableEntry> otherEntries = other.entries;

      if (thisEntries.size() != otherEntries.size())
      {
         return false;
      }

      for (YoBufferVariableEntry otherEntry : otherEntries)
      {
         YoVariable variable = otherEntry.getVariable();
         YoBufferVariableEntry thisEntry = findVariableEntry(variable.getName());

         if (thisEntry == null)
            return false;

         if (!otherEntry.epsilonEquals(thisEntry, inPoint, outPoint, epsilon))
            return false;
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
