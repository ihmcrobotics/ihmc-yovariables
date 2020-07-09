package us.ihmc.yoVariables.registry;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.function.Predicate;
import java.util.regex.Pattern;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.parameters.LongParameter;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoTools
{
   // TODO: make YoVariables use the same seperator character.
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

   public static DoubleParameter findDoubleParameter(SearchQuery query, YoRegistry registry)
   {
      return (DoubleParameter) findYoParameter(query, DoubleParameter.class::isInstance, registry);
   }

   public static BooleanParameter findBooleanParameter(SearchQuery query, YoRegistry registry)
   {
      return (BooleanParameter) findYoParameter(query, BooleanParameter.class::isInstance, registry);
   }

   public static IntegerParameter findIntegerParameter(SearchQuery query, YoRegistry registry)
   {
      return (IntegerParameter) findYoParameter(query, IntegerParameter.class::isInstance, registry);
   }

   public static LongParameter findLongParameter(SearchQuery query, YoRegistry registry)
   {
      return (LongParameter) findYoParameter(query, LongParameter.class::isInstance, registry);
   }

   public static EnumParameter<?> findEnumParameter(SearchQuery query, YoRegistry registry)
   {
      return (EnumParameter<?>) findYoParameter(query, EnumParameter.class::isInstance, registry);
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static <E extends Enum<E>> EnumParameter<E> findEnumParameter(SearchQuery query, Class<E> enumType, YoRegistry registry)
   {
      Predicate<YoParameter<?>> predicate = parameter -> parameter instanceof EnumParameter && ((EnumParameter) parameter).getEnumType() == enumType;
      return (EnumParameter<E>) findYoParameter(query, predicate, registry);
   }

   public static YoParameter<?> findYoParameter(SearchQuery query, Predicate<YoParameter<?>> predicate, YoRegistry registry)
   {
      Predicate<YoVariable<?>> yoVariablePredicate = YoVariable::isParameter;
      if (predicate != null)
         yoVariablePredicate = yoVariablePredicate.and(variable -> predicate.test(variable.getParameter()));

      YoVariable<?> result = findYoVariable(query, yoVariablePredicate, registry);
      if (result == null)
         return null;
      else
         return result.getParameter();
   }

   public static YoDouble findYoDouble(SearchQuery query, YoRegistry registry)
   {
      return (YoDouble) findYoVariable(query, YoDouble.class::isInstance, registry);
   }

   public static YoBoolean findYoBoolean(SearchQuery query, YoRegistry registry)
   {
      return (YoBoolean) findYoVariable(query, YoBoolean.class::isInstance, registry);
   }

   public static YoInteger findYoInteger(SearchQuery query, YoRegistry registry)
   {
      return (YoInteger) findYoVariable(query, YoInteger.class::isInstance, registry);
   }

   public static YoLong findYoLong(SearchQuery query, YoRegistry registry)
   {
      return (YoLong) findYoVariable(query, YoLong.class::isInstance, registry);
   }

   public static YoEnum<?> findYoEnum(SearchQuery query, YoRegistry registry)
   {
      return (YoEnum<?>) findYoVariable(query, YoEnum.class::isInstance, registry);
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static <E extends Enum<E>> YoEnum<E> findYoEnum(SearchQuery query, Class<E> enumType, YoRegistry registry)
   {
      Predicate<YoVariable<?>> predicate = variable -> variable instanceof YoEnum && ((YoEnum) variable).getEnumType() == enumType;
      return (YoEnum<E>) findYoVariable(query, predicate, registry);
   }

   public static YoVariable<?> findYoVariable(SearchQuery query, Predicate<YoVariable<?>> predicate, YoRegistry registry)
   {
      if (query.parentNameSpace == null || registry.getNameSpace().endsWith(query.parentNameSpace))
      {
         YoVariable<?> variable = registry.getVariable(query.name);
         if (variable != null)
         {
            if (predicate == null || predicate.test(variable))
               return variable;
         }
      }

      for (YoRegistry child : registry.getChildren())
      {
         YoVariable<?> variable = findYoVariable(query, predicate, child);
         if (variable != null)
            return variable;
      }

      return null;
   }

   public static List<YoVariable<?>> findYoVariables(SearchQuery query, Predicate<YoVariable<?>> predicate, YoRegistry registry,
                                                     List<YoVariable<?>> matchedVariablesToPack)
   {
      if (matchedVariablesToPack == null)
         matchedVariablesToPack = new ArrayList<>();
      if (query.parentNameSpace == null || registry.getNameSpace().endsWith(query.parentNameSpace))
      {
         YoVariable<?> variable = registry.getVariable(query.name);
         if (variable != null)
         {
            if (predicate == null || predicate.test(variable))
               matchedVariablesToPack.add(variable);
         }
      }

      for (YoRegistry child : registry.getChildren())
      {
         findYoVariables(query, predicate, child, matchedVariablesToPack);
      }
      return matchedVariablesToPack;
   }

   public static class SearchQuery
   {
      private final String parentNameSpace;
      private final String name;

      public SearchQuery(String name)
      {
         int separatorIndex = name.lastIndexOf(NAMESPACE_SEPERATOR_STRING);
         if (separatorIndex == -1)
         {
            parentNameSpace = null;
            this.name = name;
         }
         else
         {
            parentNameSpace = name.substring(0, separatorIndex);
            this.name = name.substring(separatorIndex + 1);
         }
      }

      public SearchQuery(String parentNameSpace, String name)
      {
         if (name.contains(NAMESPACE_SEPERATOR_STRING))
            throw new IllegalNameException(name + " cannot contain '" + NAMESPACE_SEPERATOR_STRING + "'.");

         this.parentNameSpace = parentNameSpace;
         this.name = name;
      }
   }

   public static void printAllVariablesIncludingDescendants(YoRegistry registry, PrintStream out)
   {
      for (YoVariable<?> var : registry.getVariables())
      {
         out.print(var.getFullNameWithNameSpace() + "\n");
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
      return splitName(fullNameSpace).get(splitName(fullNameSpace).size() - 1);
   }

   public static String joinNames(List<String> subNames)
   {
      if (subNames.stream().anyMatch(subName -> subName.contains(NAMESPACE_SEPERATOR_REGEX)))
      {
         throw new IllegalNameException("A sub name can not contain the seperator string '" + NAMESPACE_SEPERATOR + "'.");
      }
      return String.join(NAMESPACE_SEPERATOR_STRING, subNames);
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

   public static List<YoVariable<?>> searchVariablesRegex(String regularExpression, YoRegistry registry)
   {
      return searchVariablesPattern(Pattern.compile(regularExpression), registry);
   }

   public static List<YoVariable<?>> searchVariablesPattern(Pattern pattern, YoRegistry registry)
   {
      return searchVariablesPattern(pattern, registry, null);
   }

   public static List<YoVariable<?>> searchVariablesPattern(Pattern pattern, YoRegistry registry, List<YoVariable<?>> variablesToPack)
   {
      if (variablesToPack == null)
         variablesToPack = new ArrayList<>();

      for (YoVariable<?> variable : registry.getVariables())
      {
         if (pattern.matcher(variable.getName()).matches())
            variablesToPack.add(variable);
      }

      for (YoRegistry child : registry.getChildren())
      {
         searchVariablesPattern(pattern, child, variablesToPack);
      }

      return variablesToPack;
   }
}
