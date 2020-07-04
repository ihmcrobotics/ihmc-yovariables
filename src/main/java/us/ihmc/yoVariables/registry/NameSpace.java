package us.ihmc.yoVariables.registry;

import java.io.Serializable;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;

public class NameSpace implements Serializable
{
   private static final long serialVersionUID = -2584260031738121095L;

   private final String name;
   private final List<String> subNames;

   /**
    * Creates a namespace from a ordered list of names starting with the name of the root.
    *
    * @param subNames is the list of names starting with the root name.
    */
   public NameSpace(List<String> subNames)
   {
      name = joinNames(subNames);
      this.subNames = subNames;
      doChecks(name, subNames);
   }

   /**
    * Creates a namespace from a full name of the namespace. The name is the concatenation of all sub
    * names starting with the root name and separated by the {@link YoVariableTools#NAMESPACE_SEPERATOR} character.
    *
    * @param name of the namespace.
    */
   public NameSpace(String name)
   {
      this.name = name;
      subNames = splitName(name);
      doChecks(name, subNames);
   }

   public String getName()
   {
      return name;
   }

   public List<String> getSubNames()
   {
      return Collections.unmodifiableList(subNames);
   }

   public String getShortName()
   {
      return subNames.get(subNames.size() - 1);
   }

   public String getRootName()
   {
      return subNames.get(0);
   }

   public boolean isRootNameSpace()
   {
      return subNames.size() == 1;
   }

   public NameSpace getParent()
   {
      if (isRootNameSpace())
      {
         return null;
      }

      return new NameSpace(subNames.subList(0, subNames.size() - 1));
   }

   public String getNameWithRootStripped()
   {
      if (isRootNameSpace())
      {
         return null;
      }

      return joinNames(subNames.subList(1, subNames.size()));
   }

   /**
    * Checks if this nameSpace ends with the provided name.
    * <p>
    * For example, a namespace with name {@code robot.controller.module} ends with {@code module},
    * {@code controller.module}, and {@code robot.controller.module} but no other string. This means
    * this method will return false for {@code troller.module} or an empty string.
    * </p>
    *
    * @param nameToMatch is the name to check for.
    * @return whether this NameSpace ends with {@code nameToMatch}.
    */
   public boolean endsWith(String nameToMatch)
   {
      if (!name.endsWith(nameToMatch))
      {
         return false;
      }
      if (name.length() == nameToMatch.length())
      {
         return true;
      }
      return name.charAt(name.length() - nameToMatch.length() - 1) == YoVariableTools.NAMESPACE_SEPERATOR;
   }

   /**
    * Checks if this nameSpace starts with the provided name.
    * <p>
    * For example, a namespace with name {@code robot.controller.module} starts with {@code robot},
    * {@code robot.controller}, and {@code robot.controller.module} but no other string. This means
    * this method will return false for {@code robot.contro} or an empty string.
    * </p>
    *
    * @param nameToMatch is the name to check for.
    * @return whether this NameSpace starts with {@code nameToMatch}.
    */
   public boolean startsWith(String nameToMatch)
   {
      if (!name.startsWith(nameToMatch))
      {
         return false;
      }
      if (name.length() == nameToMatch.length())
      {
         return true;
      }
      return name.charAt(nameToMatch.length()) == YoVariableTools.NAMESPACE_SEPERATOR;
   }

   /**
    * Checks if this nameSpace contains the provided name.
    * <p>
    * For example, a namespace with name {@code robot.controller} contains {@code robot},
    * {@code robot.controller}, and {@code controller} but no other string. This means this method will
    * return false for {@code rob} or an empty string.
    * </p>
    *
    * @param nameToMatch is the name to check for.
    * @return whether this NameSpace contains with {@code nameToMatch}.
    */
   public boolean contains(String nameToMatch)
   {
      int startIndex = name.indexOf(nameToMatch);
      int endIndex = startIndex + nameToMatch.length();
      if (startIndex == -1)
      {
         return false;
      }
      if (name.length() == nameToMatch.length())
      {
         return true;
      }
      if (startIndex == 0)
      {
         return name.charAt(endIndex) == YoVariableTools.NAMESPACE_SEPERATOR;
      }
      if (endIndex == name.length())
      {
         return name.charAt(startIndex - 1) == YoVariableTools.NAMESPACE_SEPERATOR;
      }
      return name.charAt(endIndex) == YoVariableTools.NAMESPACE_SEPERATOR && name.charAt(startIndex - 1) == YoVariableTools.NAMESPACE_SEPERATOR;
   }

   /**
    * Will create a new namespace that is the sub-namespace of {@code this} with the provided
    * {@code nameSpaceToRemove} removed from the start.
    * <p>
    * If {@code this} does not start with the provided namespace or the provided namesace equals
    * {@code this} the method will return {@code null}.
    * </p>
    * <p>
    * If {@code nameSpaceToRemove} is {@code null} the method will return a copy of {@code this}.
    * </p>
    *
    * @param nameSpaceToRemove is the namespace to remove from the start of {@code this}.
    * @return whether this NameSpace contains with {@code nameToMatch}.
    */
   public NameSpace stripOffFromBeginning(NameSpace nameSpaceToRemove)
   {
      if (nameSpaceToRemove == null)
      {
         return new NameSpace(name);
      }
      if (equals(nameSpaceToRemove))
      {
         return null;
      }
      if (!startsWith(nameSpaceToRemove.name))
      {
         return null;
      }

      return new NameSpace(subNames.subList(nameSpaceToRemove.subNames.size(), subNames.size()));
   }

   @Override
   public int hashCode()
   {
      return name.hashCode();
   }

   @Override
   public boolean equals(Object nameSpace)
   {
      if (nameSpace == this)
      {
         return true;
      }
      if (!(nameSpace instanceof NameSpace))
      {
         return false;
      }

      return ((NameSpace) nameSpace).name.equals(name);
   }

   @Override
   public String toString()
   {
      return name;
   }

   public static NameSpace createNameSpaceFromAFullVariableName(String fullVariableName)
   {
      NameSpace parent = new NameSpace(fullVariableName).getParent();
      // TODO: this is silly we should return null here.
      return parent != null ? parent : new NameSpace("NoNameSpaceRegistry");
   }

   public static String stripOffNameSpaceToGetVariableName(String variableName)
   {
      List<String> subNames = splitName(variableName);
      return subNames.get(subNames.size() - 1);
   }

   private static List<String> splitName(String name)
   {
      return Arrays.asList(name.split(YoVariableTools.NAMESPACE_SEPERATOR_REGEX, -1));
   }

   private static String joinNames(List<String> subNames)
   {
      if (subNames.stream().anyMatch(subName -> subName.contains(YoVariableTools.NAMESPACE_SEPERATOR_REGEX)))
      {
         throw new RuntimeException("A sub name can not contain the seperator string " + YoVariableTools.NAMESPACE_SEPERATOR + ".");
      }
      return String.join(YoVariableTools.NAMESPACE_SEPERATOR_STRING, subNames);
   }

   private static void doChecks(String name, List<String> subNames)
   {
      if (subNames.stream().anyMatch(subName -> subName.isEmpty()))
      {
         throw new RuntimeException("Can not construct a namespace with an empty subname.\nNamespace: " + name);
      }

      if (!joinNames(subNames).equals(name))
      {
         throw new RuntimeException("Can not construct a namespace with inconsistent sub names.\nNamespace: " + name + "\nSub Names: " + joinNames(subNames));
      }

      if (new HashSet<>(subNames).size() != subNames.size())
      {
         throw new RuntimeException("Can not construct a namespace with duplicate sub names.\nNamespace: " + name);
      }
   }

}
