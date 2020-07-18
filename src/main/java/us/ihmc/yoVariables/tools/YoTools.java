package us.ihmc.yoVariables.tools;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.regex.Pattern;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.exceptions.IllegalNameException;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoTools
{
   public static final char NAMESPACE_SEPERATOR = '.';
   public static final String NAMESPACE_SEPERATOR_STRING = Character.toString(NAMESPACE_SEPERATOR);
   public static final String NAMESPACE_SEPERATOR_REGEX = Pattern.quote(NAMESPACE_SEPERATOR_STRING);
   public static final String ILLEGAL_CHARACTERS_STRING = "[ .*?@#$%/^&()<>,:{}'\"\\\\]";
   public static final Pattern ILLEGAL_CHARACTERS_PATTERN = Pattern.compile(ILLEGAL_CHARACTERS_STRING);

   public static void checkForIllegalCharacters(String name)
   {
      // String.matches() only matches the whole string ( as if you put ^$ around it ). Use .find() of the Matcher class instead!

      if (ILLEGAL_CHARACTERS_PATTERN.matcher(name).find())
      {
         String message = name + " contains at least one illegal character. Illegal characters: " + ILLEGAL_CHARACTERS_STRING;
         throw new IllegalNameException(message);
      }
   }

   public static void printAllVariablesIncludingDescendants(YoRegistry registry, PrintStream out)
   {
      for (YoVariable var : registry.getVariables())
      {
         out.print(var.getFullNameString() + "\n");
      }

      for (YoRegistry child : registry.getChildren())
      {
         printAllVariablesIncludingDescendants(child, out);
      }
   }

   public static void printSizeRecursively(int minVariablesToPrint, int minChildrenToPrint, YoRegistry root)
   {
      List<YoRegistry> registriesOfInterest = new ArrayList<>();
      int totalVariables = collectRegistries(minVariablesToPrint, minChildrenToPrint, root, registriesOfInterest);
      Collections.sort(registriesOfInterest, new Comparator<YoRegistry>()
      {
         @Override
         public int compare(YoRegistry o1, YoRegistry o2)
         {
            if (o1.getNumberOfVariables() == o2.getNumberOfVariables())
               return 0;
            return o1.getNumberOfVariables() > o2.getNumberOfVariables() ? -1 : 1;
         }
      });

      System.out.println("");
      LogTools.info("Printing children of " + root.getName() + " registry.");
      System.out.println("Total Number of YoVariables: " + totalVariables);
      System.out.println("Listing registries with at least " + minVariablesToPrint + " variables or at least " + minChildrenToPrint + " children.");
      System.out.println("Sorting by number of variables.");

      for (int registryIdx = 0; registryIdx < registriesOfInterest.size(); registryIdx++)
         printInfo(registriesOfInterest.get(registryIdx));

      System.out.println("");
   }

   private static int collectRegistries(int minVariablesToPrint, int minChildrenToPrint, YoRegistry registry, List<YoRegistry> registriesOfInterest)
   {
      int variables = registry.getNumberOfVariables();
      int children = registry.getChildren().size();

      if (variables >= minVariablesToPrint || children >= minChildrenToPrint)
         registriesOfInterest.add(registry);

      int totalNumberOfVariables = variables;
      for (int childIdx = 0; childIdx < children; childIdx++)
      {
         YoRegistry childRegistry = registry.getChildren().get(childIdx);
         totalNumberOfVariables += collectRegistries(minVariablesToPrint, minChildrenToPrint, childRegistry, registriesOfInterest);
      }

      return totalNumberOfVariables;
   }

   private static void printInfo(YoRegistry registry)
   {
      int variables = registry.getNumberOfVariables();
      int children = registry.getChildren().size();

      int maxPropertyLength = 17;
      String variableString = trimStringToLength("Variables: " + variables, maxPropertyLength, "...");
      String childrenString = trimStringToLength("Children: " + children, maxPropertyLength, "...");

      int maxNameLength = 70;
      String name = registry.getClass().getSimpleName() + " " + registry.getNameSpace().getName();
      name = trimStringToLength(name, maxNameLength, "...");

      System.out.println(name + "\t" + variableString + "\t" + childrenString);
   }

   private static String trimStringToLength(String original, int length, String placeholder)
   {
      int chararcters = original.length();
      int placeholderLength = placeholder.length();

      if (chararcters > length)
         return original.substring(0, length - placeholderLength) + placeholder;
      else
         return StringUtils.rightPad(original, length, " ");
   }

   public static List<String> splitName(String name)
   {
      return Arrays.asList(name.split(NAMESPACE_SEPERATOR_REGEX, -1));
   }

   public static String toShortName(String fullNameSpace)
   {
      int separatorIndex = fullNameSpace.lastIndexOf(NAMESPACE_SEPERATOR_STRING);
      if (separatorIndex == -1)
         return fullNameSpace;
      else
         return fullNameSpace.substring(separatorIndex + 1);
   }

   public static String joinNames(List<String> subNames)
   {
      if (subNames.stream().anyMatch(subName -> subName.contains(NAMESPACE_SEPERATOR_REGEX)))
      {
         throw new IllegalNameException("A sub name can not contain the seperator string '" + NAMESPACE_SEPERATOR + "'.");
      }
      return String.join(NAMESPACE_SEPERATOR_STRING, subNames);
   }

   public static void checkNameDoesNotContainSeparator(String name)
   {
      if (name.contains(NAMESPACE_SEPERATOR_STRING))
         throw new IllegalNameException("The name cannot contain '" + NAMESPACE_SEPERATOR_STRING + "'. Was: " + name);
   }

   public static void checkNameSpaceSanity(NameSpace nameSpace)
   {
      if (nameSpace.getSubNames().stream().anyMatch(subName -> subName.isEmpty()))
      {
         throw new IllegalNameException("The namespace has 1+ empty subname.\nNamespace: " + nameSpace.getName());
      }

      if (!joinNames(nameSpace.getSubNames()).equals(nameSpace.getName()))
      {
         throw new IllegalNameException("The namespace has inconsistent sub names.\nNamespace: " + nameSpace.getName() + "\nSub Names: "
               + joinNames(nameSpace.getSubNames()));
      }

      if (new HashSet<>(nameSpace.getSubNames()).size() != nameSpace.size())
      {
         throw new IllegalNameException("The namespace has duplicate sub names.\nNamespace: " + nameSpace.getName());
      }
   }

   public static NameSpace concatenate(NameSpace nameSpaceA, NameSpace nameSpaceB)
   {
      List<String> subNames = new ArrayList<>(nameSpaceA.size() + nameSpaceB.size());
      subNames.addAll(nameSpaceA.getSubNames());
      subNames.addAll(nameSpaceB.getSubNames());
      return new NameSpace(subNames);
   }

   public static NameSpace concatenate(NameSpace nameSpace, String name)
   {
      List<String> splitName = splitName(name);
      List<String> subNames = new ArrayList<>(nameSpace.size() + splitName.size());
      subNames.addAll(nameSpace.getSubNames());
      subNames.addAll(splitName);
      return new NameSpace(subNames);
   }

   public static NameSpace concatenate(String name, NameSpace nameSpace)
   {
      List<String> splitName = splitName(name);
      List<String> subNames = new ArrayList<>(splitName.size() + nameSpace.size());
      subNames.addAll(splitName);
      subNames.addAll(nameSpace.getSubNames());
      return new NameSpace(subNames);
   }

   public static NameSpace concatenate(String nameA, String nameB)
   {
      List<String> splitNameA = splitName(nameA);
      List<String> splitNameB = splitName(nameB);
      List<String> subNames = new ArrayList<>(splitNameA.size() + splitNameB.size());
      subNames.addAll(splitNameA);
      subNames.addAll(splitNameB);
      return new NameSpace(subNames);
   }

   public static boolean matchFullNameEndsWithCaseInsensitive(YoVariable yoVariable, String name)
   {
      int lastDotIndex = name.lastIndexOf(".");

      if (lastDotIndex == -1)
      {
         return yoVariable.getName().toLowerCase().equals(name.toLowerCase());
      }

      String endOfName = name.substring(lastDotIndex + 1);
      String nameSpace = name.substring(0, lastDotIndex);

      if (!endOfName.toLowerCase().equals(yoVariable.getName().toLowerCase()))
         return false;

      if (yoVariable.getRegistry() == null)
         return false;

      return yoVariable.getRegistry().getNameSpace().endsWith(nameSpace);
   }
}
