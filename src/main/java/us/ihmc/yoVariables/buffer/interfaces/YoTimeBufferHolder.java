package us.ihmc.yoVariables.buffer.interfaces;

/**
 * Minimalist interface for a class managing a time variable buffer.
 */
public interface YoTimeBufferHolder
{
   /**
    * Gets a copy of the buffer for the time variable.
    * 
    * @return the copy of the buffer for the time variable.
    */
   double[] getTimeBuffer();
}
