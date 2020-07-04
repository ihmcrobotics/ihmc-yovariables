package us.ihmc.yoVariables.registry;

import java.util.regex.Pattern;

public class YoVariableTools
{

   public static final Pattern ILLEGAL_CHARACTERS = Pattern.compile("[ .*?@#$%/^&()<>,:{}'\"\\\\]");

   public static void checkForIllegalCharacters(String name)
   {
      // String.matches() only matches the whole string ( as if you put ^$ around it ). Use .find() of the Matcher class instead!
   
      if (ILLEGAL_CHARACTERS.matcher(name).find())
      {
         String message = name + " is an invalid name for a YoVariableRegistry. A YoVariableRegistry cannot have crazy characters in them, otherwise NameSpaces"
               + " will not work.";
         throw new IllegalNameException(message);
      }
   }

   // TODO: make YoVariables use the same seperator character.
   public static final char NAMESPACE_SEPERATOR = '.';
   public static final String NAMESPACE_SEPERATOR_STRING = Character.toString(NAMESPACE_SEPERATOR);
   public static final String NAMESPACE_SEPERATOR_REGEX = Pattern.quote(NAMESPACE_SEPERATOR_STRING);

}
