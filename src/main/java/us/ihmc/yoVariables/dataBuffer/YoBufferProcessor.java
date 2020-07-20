package us.ihmc.yoVariables.dataBuffer;

import us.ihmc.yoVariables.registry.YoVariableHolder;

/**
 * A processor is a function that can be used to read and/or modify the data in a {@link YoBuffer}.
 * <p>
 * Processor is applied to a buffer as follows:
 * <ol>
 * <li>The processor is first initialized once via {@link #initialize(YoVariableHolder)} before
 * starting the procedure. The given {@code YoVariableHolder} holds the variables that can be used
 * to read from and or write into the buffer.
 * <li>The buffer goes to its in-point/out-point and reads the buffer into its {@code YoVariable}s.
 * <li>The processor is called to read from and/or write into the {@code YoVariable}s via
 * {@link #process(int, int, int)}.
 * <li>The buffer ticks forward/backward and reads the buffer into its {@code YoVariable}s. Go back
 * to the previous step until reaching the end of the buffer.
 * </ol>
 * </p>
 */
public interface YoBufferProcessor
{
   /**
    * Indicates which way the processor is to be applied.
    * 
    * @return {@code true} if the buffer is to be traveled forward, i.e. from in-point to out-point;
    *         {@code false} if the buffer is to be traveled backward, i.e. from out-point to in-point.
    */
   default boolean goForward()
   {
      return true;
   }

   /**
    * Invoked before starting the operation providing the opportunity to perform an initialization.
    * 
    * @param yoVariableHolder its contains the {@code YoVariable}s that the buffer manages and that can
    *                         be used to read from and or write into the buffer.
    */
   default void initialize(YoVariableHolder yoVariableHolder)
   {
   }

   /**
    * Defines the function to be applied to the data at each index of the buffer.
    * <p>
    * The {@code YoVariable}s are loaded from the buffer right before invoking this method. Once the
    * method has completed, the {@code YoVariable}s are then used to write into the buffer. Then the
    * buffer ticks forward/backward and the operation (read, process, write, tick) repeats until the
    * buffer has been entirely processed.
    * </p>
    * 
    * @param startIndex   the first index of the buffer to be processed. It is equal to: the in-point
    *                     when going forward, to the out-point when going backward.
    * @param endIndex     the last index of the buffer to be processed. It is equal to: the out-point
    *                     when going forward, to the in-point when going backward.
    * @param currentIndex the current buffer index to process.
    */
   void process(int startIndex, int endIndex, int currentIndex);
}
