package us.ihmc.yoVariables.buffer.interfaces;

import java.util.List;

/**
 * Interface that receives notifications of changes to a {@link KeyPointsHolder}.
 */
public interface KeyPointsChangedListener
{
   /**
    * Called after a change has been made to a {@link KeyPointsHolder}.
    * 
    * @param change an object representing the change that was done.
    * @see Change
    */
   void changed(Change change);

   /**
    * Represents a report of a single change done to a {@link KeyPointsHolder}.
    */
   public static interface Change
   {
      /**
       * Indicates that the enable flag was toggled.
       * 
       * @return {@code true} if the enable flag was changed, {@code false} otherwise.
       * @see #areKeyPointsEnabled()
       */
      boolean wasToggled();

      /**
       * Returns the current state of the enable flag.
       * 
       * @return the enable flag current value.
       */
      boolean areKeyPointsEnabled();

      /**
       * Returns the index list of key points that were added.
       * 
       * @return the new key points or an empty list if no key points were added.
       */
      List<Integer> getAddedKeyPoints();

      /**
       * Returns the index list of key points that were removed.
       * 
       * @return the key points that were removed or an empty list if no key points were removed.
       */
      List<Integer> getRemovedKeyPoints();
   }
}
