package us.ihmc.yoVariables.registry;

import java.io.Serializable;
import java.util.ArrayList;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Specifies the fully qualified namespace of a YoVariable or YoVariableRegistry. This object
 * is used to describe a YoObject's place in the tree of YoVariable registries.
 */
public class NameSpace implements Serializable
{
   private static final long serialVersionUID = -2584260031738121095L;
   private final String name;
   private final ArrayList<String> subNames;

   /**
    * Creates a new NameSpace from the fully qualified hierarchy.
    *
    * @param name String of period (.) separated names of {@link YoVariableRegistry}. The first name
    *             must be that of the root registry.
    *
    *  @throws RuntimeException if the {@code name} parameter:
    *  <ul>
    *  <li>is an empty String
    *  <li>has a period (.) as the first character
    *  <li>has a period (.) as the last character
    *  <li>2 consecutive periods (..)
    *  <li>contains duplicate names i.e. foo.bar.foo or bar.foo.foo
    *  </ul>
    */
   public NameSpace(String name)
   {
      this.name = name;
      subNames = getSubStrings(name);
      checkRepInvariant();
   }

   /**
    * Creates a new NameSpace by concatenating the list of Strings as well as adding a period between
    * each item, and passing the result to {@link #NameSpace(String)}.
    *
    * @param subNamesStartingWithRoot
    */
   public NameSpace(ArrayList<String> subNamesStartingWithRoot)
   {
      this(recreateNameFromSubNames(subNamesStartingWithRoot));
   }

   /**
    * @return List of child {@link YoVariableRegistry} names starting with the root
    */
   public ArrayList<String> getSubNames()
   {
      return subNames;
   }

   private static ArrayList<String> getSubStrings(String name)
   {
      ArrayList<String> ret = new ArrayList<String>();

      String endOfName = name;
      while (true)
      {
         int indexOfDot = endOfName.indexOf(".");
         if (indexOfDot == -1)
         {
            ret.add(endOfName);

            return ret;
         }

         String nextOne = endOfName.substring(0, indexOfDot);
         ret.add(nextOne);
         endOfName = endOfName.substring(indexOfDot + 1);
      }
   }

   private void checkRepInvariant()
   {
      String check = "";
      for (int i = 0; i < subNames.size(); i++)
      {
         String partToAdd = subNames.get(i);
         if (partToAdd.equals(""))
            throw new RuntimeException("Cannot construct NameSpace with double .. or a . at the end or beginning, or empty!");
         String subName = partToAdd;

         check = check + subName;
         if (i < subNames.size() - 1)
            check = check + ".";
      }

      if (!check.equals(name))
         throw new RuntimeException("Rep Invariant doesn't hold!! name = " + name + ", check = " + check);

      // Make sure no duplicate subnames.
      for (int i = 0; i < subNames.size() - 1; i++)
      {
         String subNameI = subNames.get(i);
         for (int j = i + 1; j < subNames.size(); j++)
         {
            String subNameJ = subNames.get(j);

            if (subNameI.equals(subNameJ))
               throw new RuntimeException("Cannot construct NameSpace with duplicate subNameSpaces! nameSpace = " + this);
         }
      }
   }

   /**
    *
    * @return Fully qualified name
    */
   public String getName()
   {
      return name;
   }

   /**
    *
    * @return Last name of the fully qualified hierarchy i.e. the leaf name of this branch
    */
   public String getShortName()
   {
      return subNames.get(subNames.size() - 1);
   }

   /**
    *
    * @return First name of the fully qualified hierarchy
    */
   public String getRootName()
   {
      return subNames.get(0);
   }

   /**
    *
    * @return Fully qualified name with the name of the root registry stripped off. Returns
    * null if there are less than 2 names in the fully qualified hierarchy.
    */
   public String getNameWithRootStripped()
   {
      if (subNames.size() < 2)
         return null;

      StringBuilder builder = new StringBuilder();
      for (int i = 1; i < subNames.size() - 1; i++)
      {
         builder.append(subNames.get(i));
         builder.append(".");
      }

      builder.append(subNames.get(subNames.size() - 1));

      return builder.toString();
   }


   private static String recreateNameFromSubNames(ArrayList<String> subNames)
   {
      StringBuilder builder = new StringBuilder();
      for (int i = 0; i < subNames.size() - 1; i++)
      {
         builder.append(subNames.get(i));
         builder.append(".");
      }

      builder.append(subNames.get(subNames.size() - 1));

      return builder.toString();
   }


   public String toString()
   {
      return name;
   }

   /**
    * Checks if the given name is in this NameSpace. Must match from the end up to a dot or the start.
    * For example "robot.controller.module" endsWith("module") and endsWith("controller.module") and endsWith("robot.controller.module")
    * but does not endsWith("bot.controller.module") or endsWith("") or anything else.
    * @param nameToMatch Name to check if this NameSpace ends with.
    * @return boolean Whether this NameSpace ends with the given name.
    */
   public boolean endsWith(String nameToMatch)
   {
      // Only true if it does end with this and, if there are more letters, that the previous one is a "."
      if (!this.name.endsWith(nameToMatch))
         return false;
      if (this.name.length() == nameToMatch.length())
         return true;
      if (this.name.length() < nameToMatch.length())
         return false;    // Defensive test. Really should never get here if the previous didn't pass.

      int index = this.name.length() - nameToMatch.length() - 1;
      char character = this.name.charAt(index);
      if (character == '.')
         return true;

      return false;
   }

   /**
    * Checks if the given name is in this NameSpace. Must match from the start up to a dot or the end.
    * For example "robot.controller.module" startsWith("robot") and startsWith("robot.controller") and startsWith("robot.controller.module")
    * but does not startsWith("robot.controller.mod") or startsWith("") or anything else.
    * @param nameToMatch Name to check if this NameSpace starts with.
    * @return boolean Whether this NameSpace starts with the given name.
    */
   public boolean startsWith(String nameToMatch)
   {
      // Only true if it does start with this and, if there are more letters, that the next one is a "."
      if (!this.name.startsWith(nameToMatch))
         return false;
      if (this.name.length() == nameToMatch.length())
         return true;
      if (this.name.length() < nameToMatch.length())
         return false;
      if (this.name.charAt(nameToMatch.length()) == '.')
         return true;

      return false;
   }

   /**
    * Checks if the given name is in this nameSpace. Must match from the start or a dot up to the end or a dot.
    * For example "robot.controller.module" contains("robot") and contains("robot.controller") and contains("robot.controller.module")
    * and contains("module") and contains("controller.module") and contains("robot.controller.module") and contains("controller")
    * but does not contains("robot.controller.mod") or contains("") or anything else.
    * @param nameToMatch Name to check if this NameSpace contains it.
    * @return boolean Whether this NameSpace contains the given name.
    */
   public boolean contains(String nameToMatch)
   {
      ArrayList<String> subNamesToMatch = getSubStrings(nameToMatch);

      if (subNamesToMatch.size() > subNames.size())
         return false;

      // Throw off the ones on the front of this subNames
      String firstSubNameToMatch = subNamesToMatch.get(0);

      int index;
      for (index = 0; index < subNames.size(); index++)
      {
         if (subNames.get(index).equals(firstSubNameToMatch))
            break;
      }

      if (subNames.size() - index < subNamesToMatch.size())
         return false;

      // Now all the rest have to match.
      for (int indexToMatch = 0; indexToMatch < subNamesToMatch.size(); indexToMatch++)
      {
         if (!subNames.get(index).equals(subNamesToMatch.get(indexToMatch)))
            return false;

         index++;
      }

      return true;
   }

   public boolean equals(Object nameSpace)
   {
      if (nameSpace == this)
         return true;
      if (!(nameSpace instanceof NameSpace))
         return false;

      NameSpace nameSpaceToCheck = (NameSpace) nameSpace;

      return nameSpaceToCheck.name.equals(this.name);
   }

   /**
    * Strips the given NameSpace off of the beginning of this NameSpace.
    *
    * @param nameSpaceToRemove ancestor of this NameSpace to remove
    *
    * @return A new NameSpace object missing the specified ancestors. Returns
    * null if
    * <ul>
    * <li>this NameSpace and {@code nameSpaceToRemove} have no names in common
    * <li>this NameSpace completely matches {@code nameSpaceToRemove}
    * </ul>
    */
   public NameSpace stripOffFromBeginning(NameSpace nameSpaceToRemove)
   {
      if (nameSpaceToRemove == null)
         return new NameSpace(this.name);

      ArrayList<String> thisSubNames = this.subNames;
      ArrayList<String> stripSubNames = nameSpaceToRemove.subNames;

      if (stripSubNames.size() >= thisSubNames.size())
         return null;

      ArrayList<String> newSubNames = new ArrayList<String>();

      for (int i = 0; i < stripSubNames.size(); i++)
      {
         if (!thisSubNames.get(i).equals(stripSubNames.get(i)))
         {
            return null;
         }
      }

      for (int i = stripSubNames.size(); i < thisSubNames.size(); i++)
      {
         newSubNames.add(thisSubNames.get(i));
      }

      return new NameSpace(newSubNames);
   }

   /**
    * Check if this is the root element
    *
    * @return true if there are no sub names.
    */
   public boolean isRootNameSpace()
   {
      return this.subNames.size() == 1;
   }

   /**
    * Get the parent namespace.
    *
    * The parent namespace is defined as this namespace with its own name stripped
    *
    * @return Parent namespace, or null if isRootNameSpace() is true.
    */
   public NameSpace getParent()
   {
      if(isRootNameSpace())
      {
         return null;
      }

      StringBuilder builder = new StringBuilder();
      for(int i = 0; i < subNames.size() - 2; i++)
      {
         builder.append(subNames.get(i));
         builder.append('.');
      }
      builder.append(subNames.get(subNames.size() - 2));
      return new NameSpace(builder.toString());

   }

   /**
    * Helper method for creating a NameSpace using the fully qualified name of a YoVariable.
    *
    * @param fullVariableName Fully qualified {@link YoVariable} name
    * @return NameSpace of the {@link YoVariableRegistry} containing that YoVariable. If {@code fullVaraibleName}
    * is not a fully qualified name, {@link #NameSpace(String)} is called with the String literal {@code "NoNameSpaceRegistry"}.
    */
   public static NameSpace createNameSpaceFromAFullVariableName(String fullVariableName)
   {
      int lastIndexOfDot = fullVariableName.lastIndexOf(".");
      if (lastIndexOfDot < 0)
      {
         return new NameSpace("NoNameSpaceRegistry");
      }

      String nameSpaceString = fullVariableName.substring(0, lastIndexOfDot);

      return new NameSpace(nameSpaceString);
   }

   /**
    * Helper method for extracting the short name of a YoVariable from its fully qualified name.
    *
    * @param variableName Fully qualified {@link YoVariable} name
    * @return Name of the just YoVariable without its hierarchy
    */
   public static String stripOffNameSpaceToGetVariableName(String variableName)
   {
      int lastIndexOfDot = variableName.lastIndexOf(".");
      if (lastIndexOfDot < 0)
      {
         return variableName;
      }

      return variableName.substring(lastIndexOfDot + 1);
   }

}
