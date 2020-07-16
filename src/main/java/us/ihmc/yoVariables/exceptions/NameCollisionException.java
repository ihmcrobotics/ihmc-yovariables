package us.ihmc.yoVariables.exceptions;

/**
 * Thrown to indicate that an operation attempted on an object resulted in a name collision.
 */
public class NameCollisionException extends RuntimeException
{
   private static final long serialVersionUID = -3153679831785972848L;

   /**
    * Creates a new {@code NameCollisionException} with no detail message.
    * 
    * @see RuntimeException#RuntimeException()
    */
   public NameCollisionException()
   {
      super();
   }

   /**
    * Creates a new {@code NameCollisionException} with the specified detail message.
    *
    * @param message the detail message.
    * @see RuntimeException#RuntimeException(String)
    */
   public NameCollisionException(String message)
   {
      super(message);
   }

   /**
    * Creates a new {@code NameCollisionException} with the specified detail message and cause.
    *
    * @param message the detail message.
    * @param cause   the cause.
    * @see RuntimeException#RuntimeException(String, Throwable)
    */
   public NameCollisionException(String message, Throwable cause)
   {
      super(message, cause);
   }

   /**
    * Creates a new {@code NameCollisionException} with the specified cause.
    *
    * @param cause the cause.
    * @see RuntimeException#RuntimeException(Throwable)
    */
   public NameCollisionException(Throwable cause)
   {
      super(cause);
   }
}
