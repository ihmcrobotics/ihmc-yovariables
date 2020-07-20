package us.ihmc.yoVariables.buffer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.yoVariables.buffer.interfaces.KeyPointsChangedListener;
import us.ihmc.yoVariables.buffer.interfaces.KeyPointsHolder;
import us.ihmc.yoVariables.buffer.interfaces.KeyPointsChangedListener.Change;

/**
 * This class allows to select and store buffer indices as key points.
 * <p>
 * A key point can be used to highlight a specific index in the buffer for any reason. The typical
 * usecase is for allowing the user to highlight particular events in the buffer and visualize them
 * via a graphical user interface.
 * </p>
 * <p>
 * This class maintains an increasing order in index value when updating its key points.
 * </p>
 */
public class KeyPointsHandler implements KeyPointsHolder
{
   /**
    * Field for convenience, only used to store an "enable" that should be associated with this object.
    */
   private boolean enableKeyPoints = false;
   /** The list of the buffer indices for each key point. */
   private final List<Integer> keyPoints = new ArrayList<>();
   /** The list of listeners to be notified of changes on this object. */
   private final List<KeyPointsChangedListener> listeners = new ArrayList<>();

   /**
    * Adds a listener to this key points handler.
    *
    * @param listener the listener for listening to changes done on the key points.
    */
   @Override
   public void addListener(KeyPointsChangedListener listener)
   {
      listeners.add(listener);
   }

   /**
    * Removes all listeners previously added to this key points handler.
    */
   public void removeListeners()
   {
      listeners.clear();
   }

   /**
    * Tries to remove a listener from this key points handler. If the listener could not be found and
    * removed, nothing happens.
    *
    * @param listener the listener to remove.
    * @return {@code true} if the listener was removed, {@code false} if the listener was not found and
    *         nothing happened.
    */
   public boolean removeListener(KeyPointsChangedListener listener)
   {
      return listeners.remove(listener);
   }

   /**
    * {@inheritDoc}
    * <p>
    * The flag is for convenience, this class does not use it for anything else besides keeping track
    * of its value. The result of this method is the change in result when calling
    * {@link #areKeyPointsEnabled()}.
    * </p>
    */
   @Override
   public void toggleKeyPoints()
   {
      enableKeyPoints = !enableKeyPoints;
      KeyPointsChange change = new KeyPointsChange(true, enableKeyPoints, null, null);
      listeners.forEach(listener -> listener.changed(change));
   }

   /**
    * Sets the "enable" flag associated to this set of key points.
    * <p>
    * The flag is for convenience, this class does not use it for anything else besides keeping track
    * of its value. The result of this method is the change in result when calling
    * {@link #areKeyPointsEnabled()}.
    * </p>
    * 
    * @param enable the new flag value.
    */
   public void enableKeyPoints(boolean enable)
   {
      if (enable != enableKeyPoints)
         toggleKeyPoints();
   }

   /**
    * {@inheritDoc}
    * <p>
    * The flag is for convenience, this class does not use it for anything else besides keeping track
    * of its value.
    * </p>
    */
   @Override
   public boolean areKeyPointsEnabled()
   {
      return enableKeyPoints;
   }

   /**
    * Toggles the presence of a key point at the given index, i.e. adds a key point if there is none or
    * removes an existing key point at the given index.
    * 
    * @param bufferIndex the index in the buffer where the key point should be toggled.
    * @return {@code true} if a key point was added, {@code false} if a key point is removed.
    */
   public boolean toggleKeyPoint(int bufferIndex)
   {
      for (int i = 0; i < keyPoints.size(); i++)
      {
         if (keyPoints.get(i) == bufferIndex)
         {
            keyPoints.remove(i);
            notifyRemovedKeyPoint(bufferIndex);
            return false;
         }

         if (keyPoints.get(i) > bufferIndex)
         {
            keyPoints.add(i, bufferIndex);
            notifyAddedKeyPoint(bufferIndex);
            return true;
         }
      }

      keyPoints.add(bufferIndex);
      return true;
   }

   /**
    * Adds a key point at the given index.
    * <p>
    * If a key point already exist at the index, nothing happens.
    * </p>
    * 
    * @param bufferIndex the index in the buffer where the key point should be added.
    * @return {@code true} if a key point was added, {@code false} if a key point already exists and
    *         nothing happened.
    */
   public boolean addKeyPoint(int bufferIndex)
   {
      for (int i = 0; i < keyPoints.size(); i++)
      {
         if (keyPoints.get(i) == bufferIndex)
         {
            return false;
         }

         if (keyPoints.get(i) > bufferIndex)
         {
            keyPoints.add(i, bufferIndex);
            notifyAddedKeyPoint(bufferIndex);
            return true;
         }
      }

      keyPoints.add(bufferIndex);
      notifyAddedKeyPoint(bufferIndex);
      return true;
   }

   /**
    * Removes a key point at the given index.
    * <p>
    * If there is no key point at the index, nothing happens.
    * </p>
    * 
    * @param bufferIndex the index in the buffer where the key point should be removed.
    * @return {@code true} if a key point was removed, {@code false} if there was no key point and
    *         nothing happened.
    */
   public boolean removeKeyPoint(int bufferIndex)
   {
      for (int i = 0; i < keyPoints.size(); i++)
      {
         if (keyPoints.get(i) == bufferIndex)
         {
            keyPoints.remove(i);
            notifyRemovedKeyPoint(bufferIndex);
            return true;
         }

         if (keyPoints.get(i) > bufferIndex)
            return false;
      }
      return false;
   }

   /**
    * Retrieves the next key point index the closest to the given index.
    * <p>
    * The next key point index is greater than the given {@code bufferIndex}. If there is no such key
    * point, the key point with smallest index is returned.
    * </p>
    * 
    * @param bufferIndex the starting point to search for the next closest key point.
    * @return the index of the next key point or {@code bufferIndex} if there are no key point
    *         registered in this handler.
    */
   public int getNextKeyPoint(int bufferIndex)
   {
      for (int i = 0; i < keyPoints.size(); i++)
      {
         if (keyPoints.get(i) > bufferIndex)
         {
            return keyPoints.get(i);
         }
      }

      if (keyPoints.size() > 0)
      {
         return keyPoints.get(0);
      }

      return bufferIndex;
   }

   /**
    * Retrieves the previous key point index the closest to the given index.
    * <p>
    * The previous key point index is smaller than the given {@code bufferIndex}. If there is no such
    * key point, the key point with largest index is returned.
    * </p>
    * 
    * @param bufferIndex the starting point to search for the previous closest key point.
    * @return the index of the previous key point or {@code bufferIndex} if there are no key point
    *         registered in this handler.
    */
   public int getPreviousKeyPoint(int bufferIndex)
   {
      for (int i = keyPoints.size() - 1; i >= 0; i--)
      {
         if (keyPoints.get(i) < bufferIndex)
         {
            return keyPoints.get(i);
         }
      }

      if (keyPoints.size() > 0)
      {
         return keyPoints.get(keyPoints.size() - 1);
      }

      return bufferIndex;
   }

   /**
    * Removes all key point which index is not in [{@code startBufferIndex}, {@code endBufferIndex}].
    * 
    * @param startBufferIndex the lower bound of the interval in which the key points are to be
    *                         preserved.
    * @param endBufferIndex   the upper bound of the interval in which the key points are to be
    *                         preserved.
    */
   public void trimKeyPoints(int startBufferIndex, int endBufferIndex)
   {
      List<Integer> removedKeyPoints = new ArrayList<>();

      for (int i = 0; i < keyPoints.size(); i++)
      {
         if (startBufferIndex < endBufferIndex)
         {
            if (keyPoints.get(i) < startBufferIndex || keyPoints.get(i) > endBufferIndex)
            {
               removedKeyPoints.add(keyPoints.remove(i));
               i--;
            }
         }
         else
         {
            if (keyPoints.get(i) < startBufferIndex && keyPoints.get(i) > endBufferIndex)
            {
               removedKeyPoints.add(keyPoints.remove(i));
               i--;
            }
         }
      }

      notifyRemovedKeyPoints(removedKeyPoints);
   }

   /**
    * Returns the indices of every key point.
    * 
    * @return the key point indices.
    */
   public List<Integer> getKeyPoints()
   {
      return keyPoints;
   }

   private void notifyAddedKeyPoint(int addedKeyPoint)
   {
      notifyAddedKeyPoints(Collections.singletonList(addedKeyPoint));
   }

   private void notifyAddedKeyPoints(List<Integer> addedKeyPoints)
   {
      KeyPointsChange change = new KeyPointsChange(false, enableKeyPoints, addedKeyPoints, null);
      listeners.forEach(listener -> listener.changed(change));
   }

   private void notifyRemovedKeyPoint(int removedKeyPoint)
   {
      notifyRemovedKeyPoints(Collections.singletonList(removedKeyPoint));
   }

   private void notifyRemovedKeyPoints(List<Integer> removedKeyPoints)
   {
      KeyPointsChange change = new KeyPointsChange(false, enableKeyPoints, null, removedKeyPoints);
      listeners.forEach(listener -> listener.changed(change));
   }

   private static class KeyPointsChange implements Change
   {
      private final boolean wasToggled;
      private final boolean areKeyPointsEnabled;
      private final List<Integer> addedKeyPoints;
      private final List<Integer> removedKeyPoints;

      public KeyPointsChange(boolean wasToggled, boolean areKeyPointsEnabled, List<Integer> addedKeyPoints, List<Integer> removedKeyPoints)
      {
         this.wasToggled = wasToggled;
         this.areKeyPointsEnabled = areKeyPointsEnabled;
         this.addedKeyPoints = addedKeyPoints == null ? Collections.emptyList() : addedKeyPoints;
         this.removedKeyPoints = removedKeyPoints == null ? Collections.emptyList() : removedKeyPoints;
      }

      @Override
      public boolean wasToggled()
      {
         return wasToggled;
      }

      @Override
      public boolean areKeyPointsEnabled()
      {
         return areKeyPointsEnabled;
      }

      @Override
      public List<Integer> getAddedKeyPoints()
      {
         return addedKeyPoints;
      }

      @Override
      public List<Integer> getRemovedKeyPoints()
      {
         return removedKeyPoints;
      }
   }
}
