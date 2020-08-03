/*
 * Copyright 2020 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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
