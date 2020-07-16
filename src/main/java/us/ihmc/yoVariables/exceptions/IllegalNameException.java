package us.ihmc.yoVariables.exceptions;

/**
 * Thrown to indicate that the name of an object contains illegal characters or is more generally
 * malformed.
 */
public class IllegalNameException extends RuntimeException
{
   private static final long serialVersionUID = 1181588237905804105L;

   /**
    * Creates a new {@code IllegalNameException} with no detail message.
    * 
    * @see RuntimeException#RuntimeException()
    */
   public IllegalNameException()
   {
      super();
   }

   /**
    * Creates a new {@code IllegalNameException} with the specified detail message.
    *
    * @param message the detail message.
    * @see RuntimeException#RuntimeException(String)
    */
   public IllegalNameException(String message)
   {
      super(message);
   }

   /**
    * Creates a new {@code IllegalNameException} with the specified detail message and cause.
    *
    * @param message the detail message.
    * @param cause   the cause.
    * @see RuntimeException#RuntimeException(String, Throwable)
    */
   public IllegalNameException(String message, Throwable cause)
   {
      super(message, cause);
   }

   /**
    * Creates a new {@code IllegalNameException} with the specified cause.
    *
    * @param cause the cause.
    * @see RuntimeException#RuntimeException(Throwable)
    */
   public IllegalNameException(Throwable cause)
   {
      super(cause);
   }
}
