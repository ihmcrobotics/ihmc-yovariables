package us.ihmc.yoVariables.dataBuffer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.yoVariables.dataBuffer.KeyPointsChangedListener.Change;

/**
 * @author jcarff
 */
public class KeyPointsHandler implements KeyPointsHolder
{
   private boolean enableKeyPoints = false;
   private final List<Integer> keyPoints = new ArrayList<>();
   private final List<KeyPointsChangedListener> listeners = new ArrayList<>();

   public void addListener(KeyPointsChangedListener listener)
   {
      listeners.add(listener);
   }

   public boolean removeListener(KeyPointsChangedListener listener)
   {
      return listeners.remove(listener);
   }

   public void removeListeners()
   {
      listeners.clear();
   }

   public void toggleKeyPoints()
   {
      enableKeyPoints = !enableKeyPoints;
      KeyPointsChange change = new KeyPointsChange(true, enableKeyPoints, null, null);
      listeners.forEach(listener -> listener.changed(change));
   }

   public void enableKeyPoints(boolean enable)
   {
      if (enable != enableKeyPoints)
         toggleKeyPoints();
   }

   public boolean areKeyPointsEnabled()
   {
      return enableKeyPoints;
   }

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

   public void trimKeyPoints(int startBufferIndex, int endBufferIndex)
   {
      List<Integer> removedKeyPoints = new ArrayList<Integer>();

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
