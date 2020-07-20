package us.ihmc.yoVariables.dataBuffer;

/**
 * Base interface for a class that manages a collection of buffer key points.
 * <p>
 * A key point can be used to highlight a specific index in the buffer for any reason. The typical
 * usecase is for allowing the user to highlight particular events in the buffer and visualize them
 * via a graphical user interface.
 * </p>
 * 
 * @see KeyPointsHandler
 */
public interface KeyPointsHolder
{
   /**
    * Toggles the "enable" flag associated to this set of key points.
    */
   void toggleKeyPoints();

   /**
    * Returns the current state of the "enable" flag associated to this set of key points.
    * 
    * @return the "enable" flag value.
    */
   boolean areKeyPointsEnabled();

   /**
    * Adds a listener to this key points holder.
    *
    * @param listener the listener for listening to changes done on the key points.
    */
   void addListener(KeyPointsChangedListener listener);
}
