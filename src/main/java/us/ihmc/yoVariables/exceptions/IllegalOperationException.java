package us.ihmc.yoVariables.exceptions;

/**
 * Thrown to indicate that an operation attempted on an object resulted in an illegal operation.
 */
public class IllegalOperationException extends RuntimeException
{
   private static final long serialVersionUID = -3153679831785972848L;

   /**
    * Creates a new {@code IllegalOperationException} with no detail message.
    * 
    * @see RuntimeException#RuntimeException()
    */
   public IllegalOperationException()
   {
      super();
   }

   /**
    * Creates a new {@code IllegalOperationException} with the specified detail message.
    *
    * @param message the detail message.
    * @see RuntimeException#RuntimeException(String)
    */
   public IllegalOperationException(String message)
   {
      super(message);
   }

   /**
    * Creates a new {@code IllegalOperationException} with the specified detail message and cause.
    *
    * @param message the detail message.
    * @param cause   the cause.
    * @see RuntimeException#RuntimeException(String, Throwable)
    */
   public IllegalOperationException(String message, Throwable cause)
   {
      super(message, cause);
   }

   /**
    * Creates a new {@code IllegalOperationException} with the specified cause.
    *
    * @param cause the cause.
    * @see RuntimeException#RuntimeException(Throwable)
    */
   public IllegalOperationException(Throwable cause)
   {
      super(cause);
   }
}
